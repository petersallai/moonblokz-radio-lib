use embassy_time::{Duration, Instant};

use crate::{MessageType, RadioMessage};

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
    wait_pool: [Option<WaitPoolItem<WAIT_POOL_SIZE>>; WAIT_POOL_SIZE],
    next_echo_request_time: Instant,
    echo_request_minimal_interval: u32,
    echo_request_additional_interval_by_neighbor: u32,
}

impl<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE> {
    pub(crate) fn new(echo_request_minimal_interval: u32, echo_request_additional_interval_by_neighbor: u32) -> Self {
        let mut result = RelayManager {
            connection_matrix: [[0; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
            connection_matrix_nodes: [0; CONNECTION_MATRIX_SIZE],
            wait_pool: [const { None }; WAIT_POOL_SIZE],
            next_echo_request_time: Instant::now(),
            echo_request_minimal_interval,
            echo_request_additional_interval_by_neighbor,
        };
        result.connection_matrix_nodes[0] = 1; // Initialize the first node as 1 (or any other default value)
        return result;
    }

    pub(crate) fn message_received(&mut self, message: RadioMessage, last_link_quality: u8) -> RelayResult {
        // find the connection matrix index for the sender node
        let mut sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id());
        if sender_index.is_none() {}

        // If message is an echo_request increment dirty counter in connection matrix
        if message.message_type() == MessageType::RequestEcho as u8 {
            if let Some(index) = sender_index {
                //set the dirty flag on connection matrix or zero the connection matrix item if it is already set
                for i in 0..CONNECTION_MATRIX_SIZE {
                    let value = self.connection_matrix[index][i];
                    //get the upper 2 bits of the value
                    let mut counter = value & 0b11000000 >> 6;
                    counter += 1;
                    if counter > 3 {
                        self.connection_matrix[index][i] = 0;
                    } else {
                        self.connection_matrix[index][i] = (counter << 6) | (last_link_quality & 0b00111111);
                    }
                }
            }
        }
        // If message is an echo_response
        if message.message_type() == MessageType::Echo as u8 {
            // find the connection matrix index for the sender node
            let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id());
            if let Some(index) = sender_index {
                //set the dirty flag on connection matrix or zero the connection matrix item if it is already set
                for i in 0..CONNECTION_MATRIX_SIZE {
                    let value = self.connection_matrix[index][i];
                    //get the upper 2 bits of the value
                    let mut counter = value & 0b11000000 >> 6;
                    counter += 1;
                    if counter > 3 {
                        self.connection_matrix[index][i] = 0;
                    } else {
                        self.connection_matrix[index][i] = (counter << 6) | (last_link_quality & 0b00111111);
                    }
                }
            }
        }
        return RelayResult::NextTimeout(Instant::now() + Duration::from_secs(self.echo_request_minimal_interval as u64));
    }
}
