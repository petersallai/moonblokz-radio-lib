use embassy_time::{Duration, Instant};
use rand_core::le;

use crate::{CONNECTION_MATRIX_SIZE, MessageType, RadioMessage};

pub(crate) enum RelayResult {
    NextTimeout(Instant),
    NextTimeoutAndSendMessage(Instant, RadioMessage),
}

struct WaitPoolItem<const CONNECTION_MATRIX_SIZE: usize> {
    message: RadioMessage,
    activation_time: u64,
    nodes_connection: [u8; CONNECTION_MATRIX_SIZE],
}

pub(crate) struct RelayManager<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> {
    connection_matrix: [[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
    connection_matrix_nodes: [u32; CONNECTION_MATRIX_SIZE],
    connected_nodes_count: usize,
    wait_pool: [Option<WaitPoolItem<WAIT_POOL_SIZE>>; WAIT_POOL_SIZE],
    next_echo_request_time: Instant,
    echo_request_minimal_interval: u32,
    echo_messages_target_interval: u8,
    own_node_id: u32,
}

impl<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE> {
    pub(crate) fn new(echo_request_minimal_interval: u32, echo_messages_target_interval: u8, own_node_id: u32) -> Self {
        let mut result = RelayManager {
            connection_matrix: [[0; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
            connection_matrix_nodes: [0; CONNECTION_MATRIX_SIZE],
            connected_nodes_count: 1, // Start with one node (own node)
            wait_pool: [const { None }; WAIT_POOL_SIZE],
            next_echo_request_time: Instant::now(),
            echo_request_minimal_interval,
            echo_messages_target_interval,
            own_node_id,
        };
        result.connection_matrix_nodes[0] = own_node_id; // Initialize the first node as own_node_id
        return result;
    }

    fn calculate_next_timeout(&self) -> Instant {
        return Instant::now() + Duration::from_secs(self.echo_request_minimal_interval as u64);
    }

    pub(crate) fn message_received(&mut self, message: RadioMessage, last_link_quality: u8) -> RelayResult {
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
            return RelayResult::NextTimeout(self.calculate_next_timeout());
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
            return RelayResult::NextTimeoutAndSendMessage(self.calculate_next_timeout(), echo_response);
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
        return RelayResult::NextTimeout(self.calculate_next_timeout());
    }
}
