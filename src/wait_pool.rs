use core::cmp::max;
use embassy_time::{Duration, Instant};
use rand_core::{RngCore, SeedableRng};
use rand_wyrand::WyRand;

use crate::{RadioMessage, RadioPacket, ScoringMatrix};

// Bitmask helpers for connection matrix cells (duplicated from relay_manager)
const QUALITY_MASK: u8 = 0b0011_1111; // lower six bits store link quality

fn calc_category(value: u8, poor_limit: u8, excellent_limit: u8) -> u8 {
    if value & QUALITY_MASK == 0 {
        0 // Zero
    } else if value < poor_limit {
        1 // Poor
    } else if value < excellent_limit {
        2 // Fair
    } else {
        3 // Excellent
    }
}

pub(crate) struct WaitPoolItem<const CONNECTION_MATRIX_SIZE: usize> {
    message: RadioMessage,
    activation_time: Instant,
    message_connections: [u8; CONNECTION_MATRIX_SIZE],
    requestor_index: Option<usize>, // Node ID of the requestor, if applicable
}

impl<const CONNECTION_MATRIX_SIZE: usize> WaitPoolItem<CONNECTION_MATRIX_SIZE> {
    pub(crate) fn calculate_score(&self, own_connections: &[u8; CONNECTION_MATRIX_SIZE], scoring_matrix: &ScoringMatrix) -> u32 {
        let mut score: u32 = 0;
        if let Some(requestor_index) = self.requestor_index {
            let network_category = calc_category(
                self.message_connections[requestor_index],
                scoring_matrix.poor_limit,
                scoring_matrix.excellent_limit,
            );
            let own_category = calc_category(own_connections[requestor_index], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
        } else {
            for i in 0..CONNECTION_MATRIX_SIZE {
                let network_category = calc_category(self.message_connections[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
                let own_category = calc_category(own_connections[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
                score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
            }
        }
        score
    }

    fn calculate_own_position(
        &self,
        item_connections: &[u8; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        scoring_matrix: &ScoringMatrix,
    ) -> u64 {
        let own_score = self.calculate_score(own_connections, scoring_matrix);
        let mut own_position = 0;
        for i in 1..CONNECTION_MATRIX_SIZE {
            if item_connections[i] > scoring_matrix.poor_limit {
                let score = self.calculate_score(&connection_matrix[i], scoring_matrix);
                if score > own_score {
                    own_position += 1;
                }
            }
        }

        own_position
    }
}

pub(crate) struct WaitPool<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> {
    items: [Option<WaitPoolItem<CONNECTION_MATRIX_SIZE>>; WAIT_POOL_SIZE],
    relay_position_delay: u64,
    scoring_matrix: ScoringMatrix,
    rng: WyRand,
}

impl<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE> {
    pub fn new(relay_position_delay: u64, scoring_matrix: ScoringMatrix, rng_seed: u64) -> Self {
        Self {
            items: [const { None }; WAIT_POOL_SIZE],
            relay_position_delay,
            scoring_matrix,
            rng: WyRand::seed_from_u64(rng_seed),
        }
    }

    pub(crate) fn contains_message_or_reply(&self, message: &RadioMessage) -> bool {
        self.items
            .iter()
            .any(|item| item.as_ref().map_or(false, |i| &i.message == message || i.message.is_reply_to(message)))
    }

    pub(crate) fn update_message(
        &mut self,
        message: &RadioMessage,
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
    ) -> bool {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if item.message == *message {
                    //log!(log::Level::Debug, "[{:?}] updating(add or) waitpool item", self.own_node_id);
                    for i in 0..CONNECTION_MATRIX_SIZE {
                        item.message_connections[i] = max(item.message_connections[i], sender_connections[i] & QUALITY_MASK);
                    }

                    if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                        //  log!(log::Level::Debug, "Message removed from wait pool");
                        *item_opt = None;
                        return true;
                    }
                    let position = item.calculate_own_position(&item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

                    item.activation_time =
                        Instant::now() + Duration::from_secs(position * self.relay_position_delay) + Duration::from_millis(self.rng.next_u64() % 300);
                    return true;
                }
            }
        }
        return false;
    }

    pub(crate) fn add_or_update_message(
        &mut self,
        message: RadioMessage,
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
        requestor_index: Option<usize>,
    ) {
        //first update existing item if present
        if self.update_message(&message, connection_matrix, own_connections, sender_connections) {
            return;
        }

        //create a new item
        let mut new_item = WaitPoolItem {
            message: message,
            activation_time: Instant::now(),
            message_connections: [0; CONNECTION_MATRIX_SIZE],
            requestor_index: requestor_index,
        };

        for i in 0..CONNECTION_MATRIX_SIZE {
            new_item.message_connections[i] = sender_connections[i] & QUALITY_MASK;
        }

        if new_item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
            return;
        }

        let position = new_item.calculate_own_position(&new_item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

        new_item.activation_time = Instant::now()
            + Duration::from_secs(position * self.relay_position_delay)
            + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));

        //find an empty spot in the waitpool

        for item_opt in self.items.iter_mut() {
            if item_opt.is_none() {
                *item_opt = Some(new_item);
                return;
            }
        }

        //no empty spot, find the item with the lowest score and replace if the new item has a higher score
        let new_score = new_item.calculate_score(own_connections, &self.scoring_matrix);
        let mut min_score = new_score;
        let mut min_index: usize = WAIT_POOL_SIZE; // Invalid index
        for (i, item_opt) in self.items.iter().enumerate() {
            if let Some(item) = item_opt {
                let score = item.calculate_score(own_connections, &self.scoring_matrix);
                if score < min_score {
                    min_score = score;
                    min_index = i;
                }
            }
        }

        if min_index < WAIT_POOL_SIZE {
            self.items[min_index] = Some(new_item);
        }
    }

    pub(crate) fn update_with_received_packet(
        &mut self,
        packet: &RadioPacket,
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
    ) {
        if let (Some(packet_sequence), Some(packet_payload_checksum)) = (packet.sequence(), packet.payload_checksum()) {
            for item_opt in self.items.iter_mut() {
                if let Some(item) = item_opt {
                    if item.requestor_index.is_some() {
                        if item.message.message_type() == packet.message_type() {
                            if let (Some(message_sequence), Some(message_payload_checksum)) = (item.message.sequence(), item.message.payload_checksum()) {
                                if message_sequence == packet_sequence && message_payload_checksum == packet_payload_checksum {
                                    if message_sequence == packet_sequence && message_payload_checksum == packet_payload_checksum {
                                        for i in 0..CONNECTION_MATRIX_SIZE {
                                            item.message_connections[i] = max(item.message_connections[i], sender_connections[i] & QUALITY_MASK);
                                        }

                                        if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                                            //  log!(log::Level::Debug, "Message removed from wait pool");
                                            *item_opt = None;
                                            return;
                                        }
                                        let position =
                                            item.calculate_own_position(&item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

                                        item.activation_time = Instant::now()
                                            + Duration::from_secs(position * self.relay_position_delay)
                                            + Duration::from_millis(self.rng.next_u64() % 300);
                                        return;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn next_activation_time(&self) -> Option<Instant> {
        self.items.iter().filter_map(|item| item.as_ref()).map(|item| item.activation_time).min()
    }

    pub(crate) fn get_next_activated_message(&mut self) -> Option<RadioMessage> {
        if let Some(next_activation_time) = self.next_activation_time() {
            if Instant::now() >= next_activation_time {
                // Process the first item that is ready, other items will be processed in the next iteration (no prioritization)
                for item_opt in self.items.iter_mut() {
                    if let Some(item) = item_opt {
                        if item.activation_time <= Instant::now() {
                            // Extract the message by taking it out and replacing with None
                            let message = core::mem::take(item_opt).unwrap().message;
                            return Some(message);
                        }
                    }
                }
            }
        }
        return None;
    }

    pub(crate) fn connection_matrix_item_changed(&mut self, index: usize) {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if let Some(requestor_idx) = item.requestor_index {
                    if requestor_idx == index {
                        // If the requestor's connection is dropped, remove the item
                        *item_opt = None;
                        continue;
                    }
                }
                item.message_connections[index] = 0;
            }
        }
    }
}
