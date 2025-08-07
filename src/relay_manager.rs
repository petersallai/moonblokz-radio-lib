use core::cmp::{max, min};
use embassy_time::{Duration, Instant};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::MessageProcessingResult;
use crate::ScoringMatrix;
use crate::{MessageType, RadioMessage};

pub(crate) enum RelayResult {
    None,
    SendMessage(RadioMessage),
    AlreadyHaveMessage,
}

fn calc_category(value: u8, poor_limit: u8, excellent_limit: u8) -> u8 {
    if value == 0 {
        0 // Zero
    } else if value < poor_limit {
        1 // Poor
    } else if value < excellent_limit {
        2 // Fair
    } else {
        3 // Excellent
    }
}

struct WaitPoolItem<const CONNECTION_MATRIX_SIZE: usize> {
    message: RadioMessage,
    activation_time: u64,
    nodes_connection: [u8; CONNECTION_MATRIX_SIZE],
}

impl<const CONNECTION_MATRIX_SIZE: usize> WaitPoolItem<CONNECTION_MATRIX_SIZE> {
    fn calculate_score(&self, own_connections: &[u8; CONNECTION_MATRIX_SIZE], scoring_matrix: &ScoringMatrix) -> u32 {
        let mut score: u32 = 0;
        for i in 0..CONNECTION_MATRIX_SIZE {
            let network_category = calc_category(self.nodes_connection[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            let own_category = calc_category(own_connections[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
        }
        score
    }
}

pub struct WaitPool<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> {
    items: [Option<WaitPoolItem<CONNECTION_MATRIX_SIZE>>; WAIT_POOL_SIZE],
}

impl<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE> {
    pub fn new() -> Self {
        Self {
            items: [const { None }; WAIT_POOL_SIZE],
        }
    }

    fn contains_message_or_reply(&self, message: &RadioMessage) -> bool {
        self.items
            .iter()
            .any(|item| item.as_ref().map_or(false, |i| &i.message == message || i.message.is_reply_to(message)))
    }
}

pub(crate) struct RelayManager<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> {
    connection_matrix: [[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
    connection_matrix_nodes: [u32; CONNECTION_MATRIX_SIZE],
    connected_nodes_count: usize,
    wait_pool: WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE>,
    next_echo_request_time: Instant,
    echo_gathering_end_time: Option<Instant>,
    echo_request_minimal_interval: u32,
    echo_messages_target_interval: u8,
    echo_gathering_timeout: u8,
    scoring_matrix: ScoringMatrix,
    own_node_id: u32,
    rng: WyRand,
}

impl<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE> {
    pub(crate) fn new(
        echo_request_minimal_interval: u32,
        echo_messages_target_interval: u8,
        echo_gathering_timeout: u8,
        scoring_matrix: ScoringMatrix,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Self {
        let mut rng = WyRand::seed_from_u64(rng_seed);
        let mut result = RelayManager {
            connection_matrix: [[0; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
            connection_matrix_nodes: [0; CONNECTION_MATRIX_SIZE],
            connected_nodes_count: 1, // Start with one node (own node)
            wait_pool: WaitPool::new(),
            next_echo_request_time: Instant::now() + Duration::from_secs(rng.next_u64() % echo_request_minimal_interval as u64),
            echo_gathering_end_time: None,
            echo_request_minimal_interval,
            echo_messages_target_interval,
            echo_gathering_timeout,
            scoring_matrix,
            own_node_id,
            rng,
        };
        result.connection_matrix_nodes[0] = own_node_id; // Initialize the first node as own_node_id
        return result;
    }

    pub(crate) fn calculate_next_timeout(&self) -> Instant {
        return min(self.next_echo_request_time, self.echo_gathering_end_time.unwrap_or(Instant::MAX));
    }

    pub(crate) fn process_timed_tasks(&mut self) -> RelayResult {
        if Instant::now() >= self.next_echo_request_time {
            let mut message_count = 2 * self.connected_nodes_count + self.connected_nodes_count * (self.connected_nodes_count - 1); // Echo request + Echo responses + Echo results
            if message_count == 0 {
                message_count = 1; // Ensure at least one message is sent
            }
            let echo_request_interval = max(
                self.echo_messages_target_interval as u32 * message_count as u32,
                self.echo_request_minimal_interval,
            ) + self.rng.next_u32() % (self.echo_gathering_timeout as u32 * 60);
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
                let send_link_quality = send_link_matrix_item & 0b00111111; // Get the link quality from the connection matrix
                let receive_link_quality = receive_link_matrix_item & 0b00111111; // Get the link quality from the connection matrix
                if (send_link_matrix_item & 0b11000000 == 0 || receive_link_matrix_item & 0b11000000 == 0)
                    && (send_link_quality != 0 || receive_link_quality != 0)
                {
                    _ = echo_result.add_echo_result_item(neighbor_node, send_link_quality, receive_link_quality);
                }
            }
            return RelayResult::SendMessage(echo_result);
        }
        return RelayResult::None;
    }

    pub(crate) fn process_received_message(&mut self, message: &RadioMessage, last_link_quality: u8) -> RelayResult {
        // find the connection matrix index for the sender node
        let mut sender_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id());
        if sender_index_opt.is_none() {
            // If the sender node is not in the connection matrix, add it
            if self.connected_nodes_count < CONNECTION_MATRIX_SIZE {
                sender_index_opt = Some(self.connected_nodes_count);
                self.connection_matrix_nodes[self.connected_nodes_count] = message.sender_node_id();
                self.connected_nodes_count += 1;
            } else {
                //find the node with the lowest link quality and replace it
                let mut lowest_quality_index = CONNECTION_MATRIX_SIZE;
                let mut lowest_quality = 255;
                for i in 0..CONNECTION_MATRIX_SIZE {
                    let value = self.connection_matrix[i][1] & 0b00111111; // Get the link quality from the connection matrix
                    if value < lowest_quality || lowest_quality_index == CONNECTION_MATRIX_SIZE {
                        lowest_quality = value;
                        lowest_quality_index = i;
                    }
                }

                // If we found a node with lower link quality, replace it
                if lowest_quality < last_link_quality {
                    sender_index_opt = Some(lowest_quality_index);
                    self.connection_matrix_nodes[lowest_quality_index] = message.sender_node_id();
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
                let value = self.connection_matrix[sender_index][i];
                //get the upper 2 bits of the value
                let mut counter = value & 0b11000000 >> 6;
                counter += 1;
                // If counter exceeds 3, zero the connection matrix cell
                if counter > 3 {
                    self.connection_matrix[sender_index][i] = 0;
                } else {
                    self.connection_matrix[sender_index][i] = (counter << 6) | (last_link_quality & 0b00111111);
                }
            }
            //send a reply echo message
            let echo_response = RadioMessage::new_echo(self.own_node_id, message.sender_node_id(), last_link_quality);
            return RelayResult::SendMessage(echo_response);
        }

        // If message is an echo_response
        if message.message_type() == MessageType::Echo as u8 {
            if let Some((target_node, link_quality)) = message.get_echo_data() {
                let target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == target_node);
                if let Some(target_index) = target_index_opt {
                    // Update the connection matrix with the link quality
                    self.connection_matrix[sender_index][target_index] = link_quality;
                }
            }
        }

        //If message is an echo_result
        if message.message_type() == MessageType::EchoResult as u8 {
            if let Some(iterator) = message.get_echo_result_data_iterator() {
                for echo_result_item in iterator {
                    let target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == echo_result_item.neighbor_node);
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
        {
            if self.wait_pool.contains_message_or_reply(message) {
                return RelayResult::AlreadyHaveMessage;
            }
        }

        return RelayResult::None;
    }

    pub(crate) fn process_processing_result(&mut self, result: MessageProcessingResult) -> RelayResult {
        match result {
            MessageProcessingResult::RequestedBlockNotFound(sequence) => return RelayResult::None,
            MessageProcessingResult::RequestedBlockFound(message) => return RelayResult::None,
            MessageProcessingResult::RequestedBlockPartFound(message, payload_checksum, part_index) => return RelayResult::None,
            MessageProcessingResult::NewBlockAdded(message) => return RelayResult::None,
            MessageProcessingResult::NewTransactionAdded(message) => return RelayResult::None,
            MessageProcessingResult::SendReplyTransaction(message) => return RelayResult::None,
            MessageProcessingResult::NewSupportAdded(message) => return RelayResult::None,
        }
    }
}
