use core::cmp::{max, min};
use embassy_time::{Duration, Instant};
use log::log;

use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::ECHO_RESPONSES_WAIT_POOL_SIZE;
use crate::MessageProcessingResult;
use crate::ScoringMatrix;
use crate::wait_pool::WaitPool;
use crate::{MessageType, RadioMessage};

// Bitmask helpers for connection matrix cells
const DIRTY_MASK: u8 = 0b1100_0000; // upper two bits used as a small counter/flags
const QUALITY_MASK: u8 = 0b0011_1111; // lower six bits store link quality
const DIRTY_SHIFT: u8 = 6; // number of bits to shift for dirty counter
const MAX_DIRTY_COUNT: u8 = 3; // after this, cells are reset to 0

pub(crate) enum RelayResult {
    None,
    SendMessage(RadioMessage),
    AlreadyHaveMessage,
}

fn empty_connections<const CONNECTION_MATRIX_SIZE: usize>() -> [u8; CONNECTION_MATRIX_SIZE] {
    [0; CONNECTION_MATRIX_SIZE]
}

pub(crate) struct RelayManager<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> {
    connection_matrix: [[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
    connection_matrix_nodes: [u32; CONNECTION_MATRIX_SIZE],
    connected_nodes_count: usize,
    wait_pool: WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE>,
    echo_responses_wait_pool: [Option<(Instant, u32, u8)>; ECHO_RESPONSES_WAIT_POOL_SIZE],
    next_echo_request_time: Instant,
    echo_gathering_end_time: Option<Instant>,
    echo_request_minimal_interval: u32,
    echo_messages_target_interval: u8,
    echo_gathering_timeout: u8,
    own_node_id: u32,
    rng: WyRand,
}

impl<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE> {
    pub(crate) fn new(
        echo_request_minimal_interval: u32,
        echo_messages_target_interval: u8,
        echo_gathering_timeout: u8,
        wait_position_delay: u8,
        scoring_matrix: ScoringMatrix,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Self {
        let mut rng = WyRand::seed_from_u64(rng_seed);
        let mut result = RelayManager {
            connection_matrix: [[0; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
            connection_matrix_nodes: [0; CONNECTION_MATRIX_SIZE],
            connected_nodes_count: 1, // Start with one node (own node)

            wait_pool: WaitPool::new(wait_position_delay as u64, scoring_matrix, rng.next_u64(), own_node_id),
            echo_responses_wait_pool: [None; ECHO_RESPONSES_WAIT_POOL_SIZE],
            next_echo_request_time: Instant::now() + Duration::from_secs(rng.next_u64() % echo_request_minimal_interval as u64),
            echo_gathering_end_time: None,
            echo_request_minimal_interval,
            echo_messages_target_interval,
            echo_gathering_timeout,
            own_node_id,
            rng,
        };
        result.connection_matrix_nodes[0] = own_node_id; // Initialize the first node as own_node_id
        result.connection_matrix[0][0] = 63; // Self-connection with max quality
        return result;
    }

    pub(crate) fn calculate_next_timeout(&self) -> Instant {
        let mut min_timeout = self.next_echo_request_time;
        if let Some(timeout) = self.echo_gathering_end_time {
            min_timeout = min(min_timeout, timeout);
        }
        if let Some(timeout) = self.wait_pool.next_activation_time() {
            min_timeout = min(min_timeout, timeout);
        }

        for i in 0..self.echo_responses_wait_pool.len() {
            if let Some((instant, _, _)) = self.echo_responses_wait_pool[i] {
                min_timeout = min(min_timeout, instant);
            }
        }
        return min_timeout;
    }

    fn add_echo_response(&mut self, node_id: u32, quality: u8) -> Option<RadioMessage> {
        let instant = Instant::now() + Duration::from_secs(self.rng.next_u64() % (self.echo_gathering_timeout as u64 * 30));
        for slot in self.echo_responses_wait_pool.iter_mut() {
            if slot.is_none() {
                *slot = Some((instant, node_id, quality));
                return None;
            }
        }

        let mut next_index = 0;
        let mut next_instant = self.echo_responses_wait_pool[0].as_ref().unwrap().0;
        for i in 1..self.echo_responses_wait_pool.len() {
            if let Some((instant, _, _)) = self.echo_responses_wait_pool[i] {
                if instant < next_instant {
                    next_instant = instant;
                    next_index = i;
                }
            }
        }

        let result = RadioMessage::new_echo(
            self.own_node_id,
            self.echo_responses_wait_pool[next_index].as_ref().unwrap().1,
            self.echo_responses_wait_pool[next_index].as_ref().unwrap().2,
        );

        self.echo_responses_wait_pool[next_index] = Some((instant, node_id, quality));
        log!(log::Level::Debug, "Echo responses wait pool full, replacing oldest entry.");

        Some(result)
    }

    pub(crate) fn process_timed_tasks(&mut self) -> RelayResult {
        if Instant::now() >= self.next_echo_request_time {
            let mut message_count = 2 * self.connected_nodes_count + self.connected_nodes_count * (self.connected_nodes_count - 1); // Echo request + Echo responses + Echo results
            if message_count == 0 {
                message_count = 1; // Ensure at least one message is sent
            }
            let echo_request_interval = self.rng.next_u32()
                % (2 * max(
                    self.echo_messages_target_interval as u32 * message_count as u32,
                    self.echo_request_minimal_interval,
                ))
                + self.rng.next_u32() % (self.echo_gathering_timeout as u32 * 60);

            self.echo_gathering_end_time = Some(Instant::now() + Duration::from_secs(self.echo_gathering_timeout as u64 * 60)); //multiply by 60 to convert minutes to seconds

            // Send an echo request to all connected nodes
            let echo_request = RadioMessage::new_request_echo(self.own_node_id);
            self.next_echo_request_time = Instant::now() + Duration::from_secs(echo_request_interval as u64);
            return RelayResult::SendMessage(echo_request);
        }

        if Instant::now() >= self.echo_gathering_end_time.unwrap_or(Instant::MAX) {
            // If echo gathering timeout has passed, reset the echo gathering end time
            self.echo_gathering_end_time = None;
            // Send an echo result message with the current connection matrix
            let mut echo_result = RadioMessage::new_echo_result(self.own_node_id);
            for i in 1..self.connected_nodes_count {
                let neighbor_node = self.connection_matrix_nodes[i];
                let send_link_matrix_item = self.connection_matrix[0][i]; // Get the link quality from the connection matrix
                let receive_link_matrix_item = self.connection_matrix[i][0]; // Get the link quality from the connection matrix
                let send_link_quality = send_link_matrix_item & QUALITY_MASK; // Get the link quality from the connection matrix
                let receive_link_quality = receive_link_matrix_item & QUALITY_MASK; // Get the link quality from the connection matrix
                if (send_link_matrix_item & DIRTY_MASK == 0 || receive_link_matrix_item & DIRTY_MASK == 0)
                    && (send_link_quality != 0 || receive_link_quality != 0)
                {
                    _ = echo_result.add_echo_result_item(neighbor_node, send_link_quality, receive_link_quality);
                }
            }
            return RelayResult::SendMessage(echo_result);
        }

        // Process wait pool items
        if let Some(next_activation_time) = self.wait_pool.next_activation_time() {
            if Instant::now() >= next_activation_time {
                // Process the first item that is ready, other items will be processed in the next iteration (no prioritization)
                for item_opt in self.wait_pool.items.iter_mut() {
                    if let Some(item) = item_opt {
                        if item.activation_time <= Instant::now() {
                            // Extract the message by taking it out and replacing with None
                            let message = core::mem::take(item_opt).unwrap().message;
                            log!(log::Level::Debug, "[{:?}] Sending relayed message", self.own_node_id);
                            return RelayResult::SendMessage(message);
                        }
                    }
                }
            }
        }

        for i in 0..self.echo_responses_wait_pool.len() {
            if let Some((instant, _, _)) = self.echo_responses_wait_pool[i] {
                if instant <= Instant::now() {
                    let response = RadioMessage::new_echo(
                        self.own_node_id,
                        self.echo_responses_wait_pool[i].as_ref().unwrap().1,
                        self.echo_responses_wait_pool[i].as_ref().unwrap().2,
                    );

                    self.echo_responses_wait_pool[i] = None; // Clear the slot
                    return RelayResult::SendMessage(response);
                }
            }
        }

        return RelayResult::None;
    }

    pub(crate) fn process_received_message(&mut self, message: &RadioMessage, last_link_quality: u8) -> RelayResult {
        if self.own_node_id == 1 {
            log::debug!("Own node id: {:?}", self.own_node_id);

            let mut nz: [u32; CONNECTION_MATRIX_SIZE] = [0; CONNECTION_MATRIX_SIZE];
            let mut nz_len = 0usize;
            for i in 0..self.connected_nodes_count {
                let id = self.connection_matrix_nodes[i];
                if id != 0 {
                    nz[nz_len] = id;
                    nz_len += 1;
                }
            }
            // Log only the top-left connected_nodes_count x connected_nodes_count submatrix
            // and mask to only show link quality (lower 6 bits), hiding dirty bits
            for r in 0..self.connected_nodes_count {
                let mut row_masked: [u8; CONNECTION_MATRIX_SIZE] = [0; CONNECTION_MATRIX_SIZE];
                for c in 0..self.connected_nodes_count {
                    row_masked[c] = self.connection_matrix[r][c] & QUALITY_MASK;
                }
                log::debug!("Node {}: {:?}", self.connection_matrix_nodes[r], &row_masked[..self.connected_nodes_count]);
            }
        }

        // find the connection matrix index for the sender node
        let mut sender_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id());
        if sender_index_opt.is_none() {
            // If the sender node is not in the connection matrix, add it
            if self.connected_nodes_count < CONNECTION_MATRIX_SIZE {
                sender_index_opt = Some(self.connected_nodes_count);
                // Initialize self-connection only; defer link qualities to echo traffic
                self.connection_matrix[self.connected_nodes_count][self.connected_nodes_count] = 63;
                self.connection_matrix_nodes[self.connected_nodes_count] = message.sender_node_id();
                self.connected_nodes_count += 1;
            } else {
                //find the node with the lowest link quality and replace it
                let mut lowest_quality_index = CONNECTION_MATRIX_SIZE;
                let mut lowest_quality = 255;
                for i in 1..CONNECTION_MATRIX_SIZE {
                    let value = self.connection_matrix[i][0] & QUALITY_MASK; // Get the link quality from the connection matrix
                    if value < lowest_quality || lowest_quality_index == CONNECTION_MATRIX_SIZE {
                        lowest_quality = value;
                        lowest_quality_index = i;
                    }
                }

                // If we found a node with lower link quality, replace it
                if lowest_quality < last_link_quality {
                    sender_index_opt = Some(lowest_quality_index);
                    self.connection_matrix_nodes[lowest_quality_index] = message.sender_node_id();
                    self.connection_matrix[lowest_quality_index] = empty_connections().clone();
                    self.connection_matrix[lowest_quality_index][lowest_quality_index] = 63;
                }
            }
        }

        let sender_index = if let Some(index) = sender_index_opt {
            index
        } else {
            //drop the message if we cannot find a place for it
            return RelayResult::None;
        };

        // If message is an echo_request increment dirty counter in connection matrix
        if message.message_type() == MessageType::RequestEcho as u8 {
            //set the dirty flag on connection matrix or zero the connection matrix item if it is already set
            for i in 0..CONNECTION_MATRIX_SIZE {
                // Skip the diagonal (sender to itself)
                if i == sender_index {
                    continue;
                }
                let value = self.connection_matrix[sender_index][i];
                // get the upper 2 bits of the value as a small counter
                let mut counter = (value & DIRTY_MASK) >> DIRTY_SHIFT;
                counter = counter.saturating_add(1);
                // If counter exceeds 3, zero the connection matrix cell
                if counter > MAX_DIRTY_COUNT {
                    self.connection_matrix[sender_index][i] = 0;
                } else {
                    // preserve existing link quality bits, update only the dirty counter
                    self.connection_matrix[sender_index][i] = (counter << DIRTY_SHIFT) | (value & QUALITY_MASK);
                }
            }
            //send a reply echo message
            //let echo_response = RadioMessage::new_echo(self.own_node_id, message.sender_node_id(), last_link_quality);
            let result_opt = self.add_echo_response(message.sender_node_id(), last_link_quality);

            if let Some(result) = result_opt {
                return RelayResult::SendMessage(result);
            }

            return RelayResult::None;
        }

        // If message is an echo_response
        if message.message_type() == MessageType::Echo as u8 {
            if let Some((target_node, link_quality)) = message.get_echo_data() {
                let target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == target_node);
                if let Some(target_index) = target_index_opt {
                    // Update the connection matrix with the link quality
                    self.connection_matrix[sender_index][0] = last_link_quality;
                    self.connection_matrix[target_index][sender_index] = link_quality;
                }
            }
        }

        //If message is an echo_result
        if message.message_type() == MessageType::EchoResult as u8 {
            if let Some(iterator) = message.get_echo_result_data_iterator() {
                for echo_result_item in iterator {
                    let mut target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == echo_result_item.neighbor_node);
                    if target_index_opt.is_none() {
                        //add to connection matrix if there is space
                        if self.connected_nodes_count < CONNECTION_MATRIX_SIZE {
                            target_index_opt = Some(self.connected_nodes_count);
                            // Initialize self-connection only; defer link qualities to echo traffic
                            self.connection_matrix[self.connected_nodes_count][self.connected_nodes_count] = 63;
                            self.connection_matrix_nodes[self.connected_nodes_count] = echo_result_item.neighbor_node;
                            self.connected_nodes_count += 1;
                        }
                    }
                    let send_link_quality = echo_result_item.send_link_quality;
                    let receive_link_quality = echo_result_item.receive_link_quality;
                    if let Some(target_index) = target_index_opt {
                        // Update the connection matrix with the link quality
                        self.connection_matrix[sender_index][target_index] = send_link_quality;
                        self.connection_matrix[target_index][sender_index] = receive_link_quality;
                    }
                }
            }
        }

        if message.message_type() == MessageType::AddBlock as u8
            || message.message_type() == MessageType::AddTransaction as u8
            || message.message_type() == MessageType::Support as u8
            || message.message_type() == MessageType::RequestFullBlock as u8
            || message.message_type() == MessageType::RequestNewMempoolItem as u8
        {
            if self.wait_pool.contains_message_or_reply(message) {
                self.wait_pool.update_message(
                    message,
                    &self.connection_matrix[0],
                    &self.connection_matrix[sender_index],
                    &self.connection_matrix,
                );
                return RelayResult::AlreadyHaveMessage;
            }
        }

        return RelayResult::None;
    }

    pub(crate) fn process_processing_result(&mut self, result: MessageProcessingResult) {
        match result {
            MessageProcessingResult::RequestedBlockNotFound(sequence) => {
                self.wait_pool.add_or_update_message(
                    RadioMessage::new_request_full_block(self.own_node_id, sequence),
                    &self.connection_matrix,
                    &self.connection_matrix[0],
                    &empty_connections::<CONNECTION_MATRIX_SIZE>(),
                );
            }
            MessageProcessingResult::RequestedBlockFound(message) => {
                self.wait_pool.add_or_update_message(
                    message,
                    &self.connection_matrix,
                    &self.connection_matrix[0],
                    &empty_connections::<CONNECTION_MATRIX_SIZE>(),
                );
            }
            MessageProcessingResult::RequestedBlockPartFound(message, _, _) => {
                self.wait_pool.add_or_update_message(
                    message,
                    &self.connection_matrix,
                    &self.connection_matrix[0],
                    &empty_connections::<CONNECTION_MATRIX_SIZE>(),
                );
            }
            MessageProcessingResult::NewBlockAdded(message) => {
                if self.own_node_id == 1 {
                    log!(log::Level::Debug, "New block added from: {:?}", message.sender_node_id());
                }

                //find sender in nodes connection list
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                if self.own_node_id == 1 {
                    log!(log::Level::Debug, "Sender index: {:?}", sender_index);
                    log!(log::Level::Debug, "Sender connections: {:?}", self.connection_matrix[sender_index]);
                }

                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections::<CONNECTION_MATRIX_SIZE>()
                };

                // Add the new block to the wait pool

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections);
            }
            MessageProcessingResult::NewTransactionAdded(message) => {
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections::<CONNECTION_MATRIX_SIZE>()
                };

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections);
            }
            MessageProcessingResult::SendReplyTransaction(message) => {
                self.wait_pool.add_or_update_message(
                    message,
                    &self.connection_matrix,
                    &self.connection_matrix[0],
                    &empty_connections::<CONNECTION_MATRIX_SIZE>(),
                );
            }
            MessageProcessingResult::NewSupportAdded(message) => {
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections::<CONNECTION_MATRIX_SIZE>()
                };

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections);
            }
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    const OWN_ID: u32 = 1;

    fn test_scoring_matrix() -> ScoringMatrix {
        // Simple matrix with uniform weights so scores are > 0
        ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0)
    }

    fn new_manager<const N: usize, const W: usize>() -> RelayManager<N, W> {
        RelayManager::new(1, 1, 1, 1, test_scoring_matrix(), OWN_ID, 42)
    }

    #[test]
    fn echo_request_increments_dirty_and_sends_reply() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();
        let sender_id = 2u32;
        let last_lq = 10u8;

        let req = RadioMessage::new_request_echo(sender_id);
        let res = rm.process_received_message(&req, last_lq);

        // With echo pooling, no immediate echo is sent
        match res {
            RelayResult::None => {}
            other => panic!("Expected None (pooled echo), got: {:?}", core::mem::discriminant(&other)),
        }

        // Sender should be inserted at index 1 (index 0 is own node)
        let sender_index = 1usize;
        // Dirty bit should be incremented and quality preserved (zero initially)
        assert_eq!(rm.connection_matrix[sender_index][0] & DIRTY_MASK, 1 << DIRTY_SHIFT);
        assert_eq!(rm.connection_matrix[sender_index][0] & QUALITY_MASK, 0);

        // An echo response should be queued in the wait pool
        let has_pooled_echo = rm
            .echo_responses_wait_pool
            .iter()
            .flatten()
            .any(|(_, node, q)| *node == sender_id && *q == last_lq);
        assert!(has_pooled_echo, "expected an echo response to be queued");
    }

    #[test]
    fn echo_updates_matrix_for_known_target() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        // Prime sender at index 1 via echo request
        let sender_id = 2u32;
        let _ = rm.process_received_message(&RadioMessage::new_request_echo(sender_id), 10);

        // Manually register a target node at index 2
        let target_id = 42u32;
        rm.connection_matrix_nodes[2] = target_id;
        rm.connected_nodes_count = 3;

        // Echo from sender -> target with link quality
        let link_q = 15u8;
        let echo = RadioMessage::new_echo(sender_id, target_id, link_q);
        let _ = rm.process_received_message(&echo, link_q);

        assert_eq!(rm.connection_matrix[1][2] & QUALITY_MASK, link_q);
    }

    #[test]
    fn echo_result_updates_both_directions() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        // Add sender at index 1
        let sender_id = 2u32;
        let _ = rm.process_received_message(&RadioMessage::new_request_echo(sender_id), 10);

        // Add neighbor at index 2
        let neighbor_id = 77u32;
        rm.connection_matrix_nodes[2] = neighbor_id;
        rm.connected_nodes_count = 3;

        // Build echo result with one item
        let mut er = RadioMessage::new_echo_result(sender_id);
        er.add_echo_result_item(neighbor_id, 20, 21).unwrap();
        let _ = rm.process_received_message(&er, 0);

        assert_eq!(rm.connection_matrix[1][2] & QUALITY_MASK, 20);
        assert_eq!(rm.connection_matrix[2][1] & QUALITY_MASK, 21);
    }

    #[test]
    fn wait_pool_adds_on_new_block() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        let msg = RadioMessage::new_add_block(3, 100, &[1, 2, 3, 4]);
        rm.process_processing_result(MessageProcessingResult::NewBlockAdded(msg));

        // Wait pool should contain the message
        let contains = rm
            .wait_pool
            .items
            .iter()
            .flatten()
            .any(|it| it.message == RadioMessage::new_add_block(3, 100, &[1, 2, 3, 4]));
        assert!(contains);
    }

    #[test]
    fn duplicate_in_wait_pool_returns_already_have_message() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        let msg1 = RadioMessage::new_add_block(3, 5, &[9, 9, 9]);
        rm.process_processing_result(MessageProcessingResult::NewBlockAdded(msg1));

        // Receiving the same (logically equal) message should return AlreadyHaveMessage
        let msg2 = RadioMessage::new_add_block(3, 5, &[9, 9, 9]);
        let res = rm.process_received_message(&msg2, 0);
        match res {
            RelayResult::AlreadyHaveMessage => {}
            other => panic!("Expected AlreadyHaveMessage, got: {:?}", core::mem::discriminant(&other)),
        }
    }
}
