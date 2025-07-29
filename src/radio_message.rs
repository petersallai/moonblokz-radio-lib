use crate::{RADIO_MAX_MESSAGE_SIZE, RADIO_PACKET_SIZE};

// Data structure for echo result items
pub struct EchoResultItem {
    pub neighbor_node: u32,
    pub send_link_quality: u8,
    pub receive_link_quality: u8,
}

impl EchoResultItem {
    /// Get the neighbor node ID
    pub fn neighbor_node(&self) -> u32 {
        self.neighbor_node
    }

    /// Get the send link quality
    pub fn send_link_quality(&self) -> u8 {
        self.send_link_quality
    }

    /// Get the receiving link quality
    pub fn receiving_link_quality(&self) -> u8 {
        self.receive_link_quality
    }
}

// Iterator for echo result data
pub struct EchoResultIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> EchoResultIterator<'a> {
    pub(crate) fn new(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for EchoResultIterator<'a> {
    type Item = EchoResultItem;

    fn next(&mut self) -> Option<Self::Item> {
        // Each item needs 6 bytes: 4 for node_id + 1 for send quality + 1 for receive quality
        if self.position + 6 > self.end_position {
            return None;
        }

        // Extract neighbor_node (u32) from 4 bytes
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.payload[self.position..self.position + 4]);
        let neighbor_node = u32::from_le_bytes(node_id_bytes);

        // Extract send and receive link qualities
        let send_link_quality = self.payload[self.position + 4];
        let receiving_link_quality = self.payload[self.position + 5];

        // Advance position to the next item
        self.position += 6;

        Some(EchoResultItem {
            neighbor_node,
            send_link_quality,
            receive_link_quality: receiving_link_quality,
        })
    }
}

#[derive(Clone, Copy)]
pub(crate) enum MessageType {
    RequestEcho = 0x01,
    Echo = 0x02,
    EchoResult = 0x03,
    RequestFullBlock = 0x04,
    RequestBlockPart = 0x05,
    AddBlock = 0x06,
    AddTransaction = 0x07,
    GetMempoolState = 0x08,
    Support = 0x09,
}

pub struct RadioMessage {
    pub(crate) payload: [u8; RADIO_MAX_MESSAGE_SIZE],
    pub(crate) length: usize,
}

impl RadioMessage {
    pub fn new_from_single_packet(packet: RadioPacket) -> RadioMessage {
        let message_type = packet.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            // Handle AddBlock and AddTransaction messages
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[0] = message_type;
            payload[0..13].copy_from_slice(&packet.data[0..13]);
            payload[13..packet.length - 2].copy_from_slice(&packet.data[15..packet.length]);

            RadioMessage {
                payload,
                length: packet.length + 1,
            }
        } else {
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[..packet.length].copy_from_slice(&packet.data[..packet.length]);
            RadioMessage {
                payload,
                length: packet.length,
            }
        }
    }

    pub(crate) fn new_empty_message() -> Self {
        // Create a new empty RadioMessage with default values
        RadioMessage {
            payload: [0u8; RADIO_MAX_MESSAGE_SIZE],
            length: 0,
        }
    }

    pub(crate) fn add_packet(&mut self, packet: &RadioPacket) {
        let total_packet_count = packet.total_packet_count();
        if total_packet_count == packet.packet_index() + 1 {
            self.payload[0..13].copy_from_slice(&packet.data[0..13]);
            self.length = 13 + (RADIO_PACKET_SIZE - 15) * (total_packet_count as usize - 1) + packet.length - 15;
        }

        let start_index = 13 + (RADIO_PACKET_SIZE - 15) * packet.packet_index() as usize;
        let end_index = match packet.packet_index() == packet.total_packet_count() - 1 {
            true => start_index + packet.length - 15,
            false => start_index + (RADIO_PACKET_SIZE - 15),
        };

        self.payload[start_index..end_index].copy_from_slice(&packet.data[15..packet.length]);
    }

    pub fn new_request_echo(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestEcho as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len() + 1].copy_from_slice(&node_id_bytes);
        RadioMessage { payload, length: 5 }
    }

    pub fn new_echo(node_id: u32, target_node_id: u32, link_quality: u8) -> Self {
        // Create a new RadioMessage with a specific message type for echo responses
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::Echo as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let target_node_id_bytes = target_node_id.to_le_bytes();
        payload[5..5 + target_node_id_bytes.len()].copy_from_slice(&target_node_id_bytes);
        payload[5 + target_node_id_bytes.len()] = link_quality;

        RadioMessage { payload, length: 10 }
    }

    pub fn new_request_full_block(node_id: u32, sequence: u32) -> Self {
        // Create a new RadioMessage with a specific message type for full block requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestFullBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);

        RadioMessage { payload, length: 9 }
    }

    pub fn new_echo_result(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo results
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::EchoResult as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage { payload, length: 5 }
    }

    pub(crate) fn add_echo_result_item(&mut self, neighbor_node: u32, send_link_quality: u8, receive_link_quality: u8) -> Result<(), ()> {
        // Add an echo result item to the message payload
        if self.length + 6 > RADIO_MAX_MESSAGE_SIZE {
            return Err(());
        }

        let start_index = self.length;
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&neighbor_node.to_le_bytes());
        self.payload[start_index..start_index + 4].copy_from_slice(&node_id_bytes);
        self.payload[start_index + 4] = send_link_quality;
        self.payload[start_index + 5] = receive_link_quality;

        self.length += 6;

        Ok(())
    }

    pub fn new_request_block_part(node_id: u32, sequence: u32, payload_checksum: u32, packet_number: u8) -> Self {
        // Create a new RadioMessage with a specific message type for block part requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestBlockPart as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        payload[13] = packet_number;

        RadioMessage { payload, length: 14 }
    }

    pub fn new_add_block(node_id: u32, sequence: u32, payload_checksum: u32, payload: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for adding blocks
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::RequestBlockPart as u8; // Reusing the same type for simplicity
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message
        full_payload[13..13 + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: 13 + payload.len(),
        }
    }

    pub fn new_add_transaction(node_id: u32, anchor_sequence: u32, payload_checksum: u32, payload: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for adding transactions
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::AddTransaction as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = anchor_sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message
        full_payload[13..13 + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: 13 + payload.len(),
        }
    }

    pub fn new_get_mempool_state(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for getting mempool state
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::GetMempoolState as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage { payload, length: 5 }
    }

    pub fn add_mempool_item(&mut self, anchor_sequence: u32, payload_checksum: u32) -> Result<(), ()> {
        // Add a mempool item to the message payload
        if self.length + 8 > RADIO_MAX_MESSAGE_SIZE {
            return Err(());
        }

        let sequence_bytes = anchor_sequence.to_le_bytes();
        let checksum_bytes = payload_checksum.to_le_bytes();
        self.payload[self.length..self.length + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        self.payload[self.length + sequence_bytes.len()..self.length + sequence_bytes.len() + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        self.length += 8;

        Ok(())
    }

    pub fn new_support(node_id: u32, sequence: u32, supporter_node: u32, signature: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for support requests
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::Support as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let supporter_node_bytes = supporter_node.to_le_bytes();
        full_payload[9..9 + supporter_node_bytes.len()].copy_from_slice(&supporter_node_bytes);
        full_payload[13..13 + signature.len()].copy_from_slice(signature);

        RadioMessage {
            payload: full_payload,
            length: 13 + signature.len(),
        }
    }

    pub fn message_type(&self) -> u8 {
        if self.payload.is_empty() {
            return 0; // Default to 0 if payload is empty
        }
        self.payload[0]
    }

    pub(crate) fn get_packet_count(&self) -> usize {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let payload_length = self.length - 13; // Exclude header
            return (payload_length + RADIO_PACKET_SIZE - 16) / (RADIO_PACKET_SIZE - 15);
        } else {
            return 1;
        }
    }

    pub(crate) fn get_packet(&self, packet_number: usize) -> Option<RadioPacket> {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let total_packets = self.get_packet_count();
            if packet_number >= total_packets {
                return None;
            }

            let start_index = 13 + packet_number * (RADIO_PACKET_SIZE - 15);
            let end_index = start_index + (RADIO_PACKET_SIZE - 15);
            let packet_data = &self.payload[start_index..end_index.min(self.length)];
            let packet_header = &self.payload[0..13]; // First 13 bytes are the header

            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..13].copy_from_slice(packet_header);
            data[13] = total_packets as u8; // Total packet count in the header
            data[14] = packet_number as u8; // Packet number in the header
            data[15..packet_data.len() + 15].copy_from_slice(packet_data);

            Some(RadioPacket {
                data,
                length: packet_data.len() + 15,
            })
        } else {
            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..self.length].copy_from_slice(&self.payload[0..self.length]);
            Some(RadioPacket { data, length: self.length })
        }
    }

    pub fn sender_node_id(&self) -> u32 {
        // Extract the sender node ID from the message payload
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.payload[1..5]);
        u32::from_le_bytes(node_id_bytes)
    }

    pub fn length(&self) -> usize {
        self.length
    }

    pub(crate) fn get_echo_data(&self) -> Option<(u32, u8)> {
        // Extract echo data from the message payload
        if self.message_type() != MessageType::Echo as u8 {
            return None;
        }
        if self.length < 6 {
            return None; // Not enough data for echo
        }

        let link_quality = self.payload[5];
        let mut sender_node_id_bytes = [0u8; 4];
        sender_node_id_bytes.copy_from_slice(&self.payload[1..5]);
        let sender_node_id = u32::from_le_bytes(sender_node_id_bytes);

        Some((sender_node_id, link_quality))
    }

    /// Get an iterator over echo result data entries.
    /// Each entry contains neighbor node ID, send link quality, and receiving link quality.
    ///
    /// # Example
    /// ```rust
    /// // Assuming you have a RadioMessage with EchoResult message type
    /// if let Some(iterator) = message.get_echo_result_data_iterator() {
    ///     for item in iterator {
    ///         println!("Neighbor: {}, Send Quality: {}, Receive Quality: {}",
    ///                  item.neighbor_node(), item.send_link_quality(), item.receiving_link_quality());
    ///     }
    /// }
    /// ```
    pub(crate) fn get_echo_result_data_iterator(&self) -> Option<EchoResultIterator> {
        if self.message_type() != MessageType::EchoResult as u8 {
            return None;
        }
        if self.length < 5 {
            return None; // Not enough data for echo result
        }

        Some(EchoResultIterator::new(&self.payload, 5, self.length))
    }

    pub fn get_add_transaction_data(&self) -> Option<(u32, u32, &[u8])> {
        // Extract transaction data from the message payload
        if self.message_type() != MessageType::AddTransaction as u8 {
            return None;
        }
        if self.length < 13 {
            return None; // Not enough data for transaction
        }

        let mut anchor_sequence_bytes = [0u8; 4];
        anchor_sequence_bytes.copy_from_slice(&self.payload[5..9]);
        let anchor_sequence = u32::from_le_bytes(anchor_sequence_bytes);

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[9..13]);
        let checksum = u32::from_le_bytes(checksum_bytes);

        Some((anchor_sequence, checksum, &self.payload[13..self.length]))
    }
}

pub struct RadioPacket {
    pub(crate) data: [u8; RADIO_PACKET_SIZE],
    pub(crate) length: usize,
}

impl RadioPacket {
    pub(crate) fn message_type(&self) -> u8 {
        self.data[0]
    }

    pub(crate) fn total_packet_count(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < 15 {
                return 0; // Not enough data for packet count
            }
            return self.data[13];
        } else {
            return 1; // For other message types, always 1 packet
        }
    }

    pub(crate) fn packet_index(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < 15 {
                return 0; // Not enough data for packet index
            }
            return self.data[14];
        } else {
            return 0; // For other message types, always 0
        }
    }
}
