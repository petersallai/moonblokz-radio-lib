use core::cmp::max;
use embassy_time::{Duration, Instant};
use log::log;
use rand_core::{RngCore, SeedableRng};
use rand_wyrand::WyRand;

use crate::{RadioMessage, ScoringMatrix};

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
    pub(crate) message: RadioMessage,
    pub(crate) activation_time: Instant,
    pub(crate) nodes_connection: [u8; CONNECTION_MATRIX_SIZE],
}

impl<const CONNECTION_MATRIX_SIZE: usize> WaitPoolItem<CONNECTION_MATRIX_SIZE> {
    pub(crate) fn calculate_score(&self, own_connections: &[u8; CONNECTION_MATRIX_SIZE], scoring_matrix: &ScoringMatrix) -> u32 {
        let mut score: u32 = 0;
        for i in 0..CONNECTION_MATRIX_SIZE {
            let network_category = calc_category(self.nodes_connection[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            let own_category = calc_category(own_connections[i], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            /*             log!(
                log::Level::Debug,
                "[{i}] Network category: {}, Own category: {}, additional score: {}",
                network_category,
                own_category,
                scoring_matrix.matrix[network_category as usize][own_category as usize]
            );*/
            score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
        }
        score
    }

    pub(crate) fn calculate_own_position(
        &self,
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        scoring_matrix: &ScoringMatrix,
    ) -> u64 {
        let own_score = self.calculate_score(own_connections, scoring_matrix);
        //log!(log::Level::Debug, "Own score: {}", own_score);
        let mut own_position = 0;
        for i in 1..CONNECTION_MATRIX_SIZE {
            let score = self.calculate_score(&connection_matrix[i], scoring_matrix);
            //  log!(log::Level::Debug, "[{i}] Other score: {}", score);
            if score > own_score {
                own_position += 1;
            }
        }

        own_position
    }
}

pub(crate) struct WaitPool<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> {
    pub(crate) items: [Option<WaitPoolItem<CONNECTION_MATRIX_SIZE>>; WAIT_POOL_SIZE],
    relay_position_delay: u64,
    pub(crate) scoring_matrix: ScoringMatrix,
    rng: WyRand,
    //TODO:Remove
    own_node_id: u32,
}

impl<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE> {
    pub fn new(relay_position_delay: u64, scoring_matrix: ScoringMatrix, rng_seed: u64, own_node_id: u32) -> Self {
        Self {
            items: [const { None }; WAIT_POOL_SIZE],
            relay_position_delay,
            scoring_matrix,
            rng: WyRand::seed_from_u64(rng_seed),
            own_node_id: own_node_id,
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
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
    ) {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if item.message == *message {
                    log!(log::Level::Debug, "[{:?}] updating waitpool item", self.own_node_id);
                    for i in 0..CONNECTION_MATRIX_SIZE {
                        item.nodes_connection[i] = max(item.nodes_connection[i], sender_connections[i] & QUALITY_MASK);
                    }
                    log!(
                        log::Level::Debug,
                        "Own score: {:?}",
                        item.calculate_score(own_connections, &self.scoring_matrix)
                    );
                    if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                        log!(log::Level::Debug, "Message removed from wait pool");
                        *item_opt = None;
                    } else {
                        item.activation_time = Instant::now()
                            + Duration::from_secs(
                                item.calculate_own_position(sender_connections, own_connections, connection_matrix, &self.scoring_matrix)
                                    * self.relay_position_delay,
                            )
                            + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));
                    }
                    return;
                }
            }
        }
    }

    pub(crate) fn add_or_update_message(
        &mut self,
        message: RadioMessage,
        connection_matrix: &[[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        own_connections: &[u8; CONNECTION_MATRIX_SIZE],
        sender_connections: &[u8; CONNECTION_MATRIX_SIZE],
    ) {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if item.message == message {
                    log!(log::Level::Debug, "[{:?}] updating(add or) waitpool item", self.own_node_id);
                    for i in 0..CONNECTION_MATRIX_SIZE {
                        item.nodes_connection[i] = max(item.nodes_connection[i], sender_connections[i] & QUALITY_MASK);
                    }

                    log!(
                        log::Level::Debug,
                        "Own score: {:?}",
                        item.calculate_score(own_connections, &self.scoring_matrix)
                    );
                    if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                        log!(log::Level::Debug, "Message removed from wait pool");
                        *item_opt = None;
                        return;
                    }
                    let position = item.calculate_own_position(sender_connections, own_connections, connection_matrix, &self.scoring_matrix);

                    item.activation_time = Instant::now()
                        + Duration::from_secs(position * self.relay_position_delay)
                        + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));
                    return;
                }
            } else {
                log!(log::Level::Debug, "[{}] Message added to wait pool", self.own_node_id);

                log!(log::Level::Debug, "sender connections: {:?}", sender_connections);
                let mut new_item = WaitPoolItem {
                    message,
                    activation_time: Instant::now(),
                    nodes_connection: [0; CONNECTION_MATRIX_SIZE],
                };

                for i in 0..CONNECTION_MATRIX_SIZE {
                    new_item.nodes_connection[i] = sender_connections[i] & QUALITY_MASK;
                }

                log!(
                    log::Level::Debug,
                    "Own score: {:?}",
                    new_item.calculate_score(own_connections, &self.scoring_matrix)
                );
                if new_item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                    log!(log::Level::Debug, "Message not added to the pool");
                    return;
                }

                let position = new_item.calculate_own_position(sender_connections, own_connections, connection_matrix, &self.scoring_matrix);

                log!(log::Level::Debug, "position: {:?}", position);
                new_item.activation_time = Instant::now()
                    + Duration::from_secs(position * self.relay_position_delay)
                    + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));

                *item_opt = Some(new_item);
                return;
            }
        }

        let mut new_item = WaitPoolItem {
            message,
            activation_time: Instant::now(),
            nodes_connection: [0; CONNECTION_MATRIX_SIZE],
        };

        for i in 0..CONNECTION_MATRIX_SIZE {
            new_item.nodes_connection[i] = sender_connections[i] & QUALITY_MASK;
        }

        let position = new_item.calculate_own_position(sender_connections, own_connections, connection_matrix, &self.scoring_matrix);
        new_item.activation_time = Instant::now()
            + Duration::from_secs(position * self.relay_position_delay)
            + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));

        let new_score = new_item.calculate_score(own_connections, &self.scoring_matrix);
        let mut min_score = new_score;
        let mut min_index: usize = 0;
        for (i, item_opt) in self.items.iter().enumerate() {
            if let Some(item) = item_opt {
                let score = item.calculate_score(own_connections, &self.scoring_matrix);
                if score < min_score {
                    min_score = score;
                    min_index = i;
                }
            }
        }

        if new_score > min_score {
            self.items[min_index] = Some(new_item);
        }
    }

    pub(crate) fn next_activation_time(&self) -> Option<Instant> {
        self.items.iter().filter_map(|item| item.as_ref()).map(|item| item.activation_time).min()
    }
}
