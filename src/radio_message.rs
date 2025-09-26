use crate::{RADIO_MAX_MESSAGE_SIZE, RADIO_MAX_PACKET_COUNT, RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE, RADIO_MULTI_PACKET_PACKET_HEADER_SIZE, RADIO_PACKET_SIZE};
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

fn crc32c(data: &[u8]) -> u32 {
    const POLY: u32 = 0x82F63B78;
    let mut crc: u32 = 0xFFFF_FFFF;

    for &b in data {
        let mut cur = (crc ^ (b as u32)) & 0xFF;
        for _ in 0..8 {
            let mask = (cur & 1).wrapping_neg();
            cur = (cur >> 1) ^ (POLY & mask);
        }
        crc = (crc >> 8) ^ cur;
    }

    crc ^ 0xFFFF_FFFF
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

// Data structure for request block part item (single packet index)
pub struct RequestBlockPartItem {
    pub packet_index: u8,
}

impl RequestBlockPartItem {
    /// Get the requested packet index
    pub fn packet_index(&self) -> u8 {
        self.packet_index
    }
}

// Iterator for request block part indices
pub struct RequestBlockPartIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> RequestBlockPartIterator<'a> {
    pub(crate) fn new(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for RequestBlockPartIterator<'a> {
    type Item = RequestBlockPartItem;

    fn next(&mut self) -> Option<Self::Item> {
        if self.position >= self.end_position {
            return None;
        }
        let idx = self.payload[self.position];
        self.position += 1;

        Some(RequestBlockPartItem { packet_index: idx })
    }
}

#[derive(Clone, Copy)]
pub enum MessageType {
    RequestEcho = 0x01,
    Echo = 0x02,
    EchoResult = 0x03,
    RequestFullBlock = 0x04,
    RequestBlockPart = 0x05,
    AddBlock = 0x06,
    AddTransaction = 0x07,
    RequestNewMempoolItem = 0x08,
    Support = 0x09,
}

#[derive(Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioMessage {
    pub payload: [u8; RADIO_MAX_MESSAGE_SIZE],
    pub length: usize,
    pub packets_to_send: Option<[bool; RADIO_MAX_PACKET_COUNT]>,
}

impl RadioMessage {
    pub fn from_single_packet(packet: RadioPacket) -> RadioMessage {
        let message_type = packet.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            // Handle AddBlock and AddTransaction messages
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(&packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);
            // Skip the 2 per-packet meta bytes (total count, index) right after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
            payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + (packet.length - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE)]
                .copy_from_slice(&packet.data[RADIO_MULTI_PACKET_PACKET_HEADER_SIZE..packet.length]);

            RadioMessage {
                payload,
                // RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + actual payload (skipping per-packet header)
                length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + (packet.length - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE),
                packets_to_send: None,
            }
        } else {
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[..packet.length].copy_from_slice(&packet.data[..packet.length]);
            RadioMessage {
                payload,
                length: packet.length,
                packets_to_send: None,
            }
        }
    }

    pub(crate) fn new() -> Self {
        // Create a new empty RadioMessage with default values
        RadioMessage {
            payload: [0u8; RADIO_MAX_MESSAGE_SIZE],
            length: 0,
            packets_to_send: None,
        }
    }

    pub(crate) fn add_packet(&mut self, packet: &RadioPacket) {
        let total_packet_count = packet.total_packet_count();
        if total_packet_count == packet.packet_index() + 1 {
            self.payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(&packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);
            self.length = RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
                + (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE) * (total_packet_count as usize - 1)
                + packet.length
                - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        }

        let start_index = RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE) * packet.packet_index() as usize;
        let end_index = match packet.packet_index() == packet.total_packet_count() - 1 {
            true => start_index + packet.length - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE,
            false => start_index + (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE),
        };

        self.payload[start_index..end_index].copy_from_slice(&packet.data[RADIO_MULTI_PACKET_PACKET_HEADER_SIZE..packet.length]);
    }

    pub fn new_request_echo(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestEcho as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len() + 1].copy_from_slice(&node_id_bytes);
        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    pub fn new_echo(node_id: u32, target_node_id: u32, link_quality: u8) -> Self {
        // Create a new RadioMessage with a specific message type for echo responses
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::Echo as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let target_node_id_bytes = target_node_id.to_le_bytes();
        payload[5..5 + target_node_id_bytes.len()].copy_from_slice(&target_node_id_bytes);
        payload[5 + target_node_id_bytes.len()] = link_quality;

        RadioMessage {
            payload,
            length: 10,
            packets_to_send: None,
        }
    }

    pub fn new_request_full_block(node_id: u32, sequence: u32) -> Self {
        // Create a new RadioMessage with a specific message type for full block requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestFullBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);

        RadioMessage {
            payload,
            length: 9,
            packets_to_send: None,
        }
    }

    pub fn new_echo_result(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo results
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::EchoResult as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    pub(crate) fn add_echo_result_item(&mut self, neighbor_node: u32, send_link_quality: u8, receive_link_quality: u8) -> Result<(), ()> {
        // Add an echo result item to the message payload (each item is 6 bytes)
        if self.length + 6 > RADIO_PACKET_SIZE {
            //the message must fit into a single packet
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

    pub(crate) fn new_request_block_part(node_id: u32, sequence: u32, payload_checksum: u32) -> Self {
        // Create a new RadioMessage with a specific message type for block part requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestBlockPart as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        payload[13] = 0; //current index count set to 0

        RadioMessage {
            payload,
            length: 14,
            packets_to_send: None,
        }
    }

    pub(crate) fn add_packet_index_to_request_block_part(&mut self, packet_index: u8) -> Result<(), ()> {
        // Add a packet index to the request block part message
        if self.message_type() != MessageType::RequestBlockPart as u8 {
            return Err(());
        }

        // The message must fit into a single radio packet
        if self.length + 1 > RADIO_PACKET_SIZE {
            return Err(());
        }
        self.payload[self.payload[13] as usize + 14] = packet_index;

        self.payload[13] += 1;
        self.length += 1;

        Ok(())
    }

    pub fn new_add_block(node_id: u32, sequence: u32, payload: &[u8]) -> Self {
        // Calculate payload checksum
        let payload_checksum = crc32c(payload);

        // Create a new RadioMessage with a specific message type for adding blocks
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::AddBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len(),
            packets_to_send: None,
        }
    }

    pub fn new_add_block_with_packet_list(node_id: u32, sequence: u32, payload: &[u8], packets_to_send: [bool; RADIO_MAX_PACKET_COUNT]) -> Self {
        let mut new_message = Self::new_add_block(node_id, sequence, payload);
        new_message.packets_to_send = Some(packets_to_send);
        new_message
    }

    pub fn add_packet_list(&mut self, packets_to_send: [bool; RADIO_MAX_PACKET_COUNT]) {
        if self.message_type() != MessageType::AddBlock as u8 && self.message_type() != MessageType::AddTransaction as u8 {
            return; // Only AddBlock and AddTransaction messages can have packet lists
        }
        self.packets_to_send = Some(packets_to_send);
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

        // Copy the actual payload data into the message after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len(),
            packets_to_send: None,
        }
    }

    pub fn new_get_mempool_state(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for getting mempool state
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestNewMempoolItem as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    pub fn add_mempool_item(&mut self, anchor_sequence: u32, payload_checksum: u32) -> Result<(), ()> {
        // Add a mempool item to the message payload (8 bytes per item)
        if self.message_type() != MessageType::RequestNewMempoolItem as u8 {
            return Err(());
        }

        // The message must fit into a single radio packet
        if self.length + 8 > RADIO_PACKET_SIZE {
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
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let supporter_node_bytes = supporter_node.to_le_bytes();
        full_payload[9..9 + supporter_node_bytes.len()].copy_from_slice(&supporter_node_bytes);
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + signature.len()].copy_from_slice(signature);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + signature.len(),
            packets_to_send: None,
        }
    }

    pub fn message_type(&self) -> u8 {
        if self.length == 0 {
            return 0; // Default to 0 if message has no logical length
        }
        self.payload[0]
    }

    fn get_total_packet_count(&self) -> usize {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let payload_length = self.length.saturating_sub(RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE); // Exclude header
            if payload_length == 0 {
                return 0;
            }
            let packet_count = core::cmp::min(
                (payload_length + RADIO_PACKET_SIZE - (RADIO_MULTI_PACKET_PACKET_HEADER_SIZE + 1))
                    / (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE),
                RADIO_MAX_PACKET_COUNT,
            );
            return packet_count;
        } else {
            return 1;
        }
    }

    pub(crate) fn get_packet_count(&self) -> usize {
        let mut packet_count = self.get_total_packet_count();
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            if let Some(packets_to_send) = self.packets_to_send {
                let unfiltered_packet_count = packet_count;

                for i in 0..unfiltered_packet_count {
                    if !packets_to_send[i] {
                        packet_count -= 1;
                    }
                }
            }
        }
        return packet_count;
    }

    pub(crate) fn get_packet(&self, input_packet_number: usize) -> Option<RadioPacket> {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let total_packets = self.get_total_packet_count();
            if input_packet_number >= total_packets {
                return None;
            }

            let mut packet_number = input_packet_number;

            if let Some(packets_to_send) = self.packets_to_send {
                let mut actual_packet_number = 0;
                let mut found_packets = 0;

                for i in 0..total_packets {
                    if packets_to_send[i] {
                        if found_packets == packet_number {
                            actual_packet_number = i;
                            found_packets += 1;
                            break;
                        }
                        found_packets += 1;
                    }
                }
                if found_packets <= input_packet_number {
                    return None; // Not enough packets to satisfy the request
                }
                packet_number = actual_packet_number;
            }

            let start_index = RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + packet_number * (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE);
            let end_index = start_index + (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE);
            let packet_data = &self.payload[start_index..end_index.min(self.length)];
            let packet_header = &self.payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]; // First RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes are the multi-packet message header

            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(packet_header);
            // We use the real (unfiltered) total count and packet index here because the receiver needs to reconstruct the full message
            data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] = total_packets as u8; // total count
            data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + 1] = packet_number as u8; // packet index
            data[RADIO_MULTI_PACKET_PACKET_HEADER_SIZE..packet_data.len() + RADIO_MULTI_PACKET_PACKET_HEADER_SIZE].copy_from_slice(packet_data);

            Some(RadioPacket {
                data,
                length: packet_data.len() + RADIO_MULTI_PACKET_PACKET_HEADER_SIZE,
            })
        } else {
            if input_packet_number != 0 {
                return None; // Only one packet available for non multi-packet messages
            }
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

    pub fn set_sender_node_id(&mut self, node_id: u32) {
        let node_id_bytes = node_id.to_le_bytes();
        self.payload[1..5].copy_from_slice(&node_id_bytes);
    }

    pub fn length(&self) -> usize {
        self.length
    }

    pub(crate) fn get_echo_data(&self) -> Option<(u32, u8)> {
        // Extract echo data from the message payload
        if self.message_type() != MessageType::Echo as u8 {
            return None;
        }
        if self.length < 10 {
            return None; // Not enough data for echo
        }

        // Target node id is at bytes 5..9, link quality at byte 9
        let mut target_node_id_bytes = [0u8; 4];
        target_node_id_bytes.copy_from_slice(&self.payload[5..9]);
        let target_node_id = u32::from_le_bytes(target_node_id_bytes);
        let link_quality = self.payload[9];

        Some((target_node_id, link_quality))
    }

    /// Get an iterator over echo result data entries.
    /// Each entry contains neighbor node ID, send link quality, and receiving link quality.
    ///
    /// # Example
    /// ```rust,ignore
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

    pub fn sequence(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8
            && self.message_type() != MessageType::RequestFullBlock as u8
            && self.message_type() != MessageType::Support as u8
            && self.message_type() != MessageType::RequestBlockPart as u8
        {
            return None; // Only AddBlock and AddTransaction messages have a sequence
        }

        // Extract the sequence number from the message payload
        if self.length < 9 {
            return None; // Not enough data for sequence
        }
        let mut sequence_bytes = [0u8; 4];
        sequence_bytes.copy_from_slice(&self.payload[5..9]);
        Some(u32::from_le_bytes(sequence_bytes))
    }

    pub fn is_reply_to(&self, message: &RadioMessage) -> bool {
        if self.message_type() == MessageType::Echo as u8 {
            if message.message_type() != MessageType::RequestEcho as u8 {
                return false; // Only Echo messages can reply to RequestEcho
            }
            // For Echo messages, check if the sender node ID matches
            return self.sender_node_id() == message.sender_node_id();
        }

        if self.message_type() == MessageType::AddBlock as u8 {
            if message.message_type() != MessageType::RequestFullBlock as u8 {
                return false; // Only AddBlock messages can reply to RequestFullBlock
            }
            return message.sequence() == self.sequence();
        }

        if self.message_type() == MessageType::AddTransaction as u8 {
            if message.message_type() != MessageType::RequestNewMempoolItem as u8 {
                return false; // Only AddTransaction messages can reply to GetMempoolState
            }

            let (anchor_sequence, checksum, _) = self.get_add_transaction_data().unwrap();

            if let Some(mempool_data_iterator) = message.get_mempool_data_iterator() {
                // Iterate through the mempool data entries
                // Each entry contains anchor sequence and transaction payload checksum
                // Check if the mempool item matches the current message
                for mempool_item in mempool_data_iterator {
                    // Check if the mempool item matches the current message
                    if mempool_item.anchor_sequence() == anchor_sequence && mempool_item.transaction_payload_checksum() == checksum {
                        return false; // Found a matching mempool item
                    }
                }
                // If no matching mempool item was found the message can be considered a reply
                // to the GetMempoolState request
                return true;
            }
        }
        // For other message types, we do not consider them as replies
        return false;
    }

    /// Get an iterator over mempool data entries.
    /// Each entry contains anchor sequence and transaction payload checksum.
    ///
    /// # Example
    /// ```rust,ignore
    /// // Assuming you have a RadioMessage with GetMempoolState message type
    /// if let Some(iterator) = message.get_mempool_data_iterator() {
    ///     for item in iterator {
    ///         println!("Anchor Sequence: {}, Transaction Checksum: {}",
    ///                  item.anchor_sequence(), item.transaction_payload_checksum());
    ///     }
    /// }
    /// ```
    pub(crate) fn get_mempool_data_iterator(&self) -> Option<MempoolIterator> {
        if self.message_type() != MessageType::RequestNewMempoolItem as u8 {
            return None;
        }
        if self.length < 5 {
            return None; // Not enough data for mempool state (at least message type + sender_node)
        }

        Some(MempoolIterator::new(&self.payload, 5, self.length))
    }

    pub(crate) fn payload_checksum(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8 && self.message_type() != MessageType::AddTransaction as u8 {
            return None; // Only AddBlock and AddTransaction messages have a payload checksum
        }
        if self.length < 13 {
            return None; // Not enough data for checksum
        }

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[9..13]);
        Some(u32::from_le_bytes(checksum_bytes))
    }

    /// Get an iterator over RequestBlockPart packet indices.
    /// Layout: [0]=type, [1..5)=sender, [5..9)=sequence, [9..13)=checksum, [13]=count, [14..14+count)=indices
    pub fn get_request_block_part_iterator(&self) -> Option<RequestBlockPartIterator> {
        if self.message_type() != MessageType::RequestBlockPart as u8 {
            return None;
        }
        // need at least up to count byte present
        if self.length < 14 {
            return None;
        }
        let count = self.payload[13] as usize;
        // Start of indices immediately after count byte
        let start = 14usize;
        // Use count as authoritative; cap to a single radio packet size for safety
        let end = (start + count).min(RADIO_PACKET_SIZE);
        Some(RequestBlockPartIterator::new(&self.payload, start, end))
    }
}

// Data structure for mempool items
pub struct MempoolItem {
    pub anchor_sequence: u32,
    pub transaction_payload_checksum: u32,
}

impl MempoolItem {
    /// Get the anchor sequence
    pub fn anchor_sequence(&self) -> u32 {
        self.anchor_sequence
    }

    /// Get the transaction payload checksum
    pub fn transaction_payload_checksum(&self) -> u32 {
        self.transaction_payload_checksum
    }
}

// Iterator for mempool data
pub struct MempoolIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> MempoolIterator<'a> {
    pub(crate) fn new(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for MempoolIterator<'a> {
    type Item = MempoolItem;

    fn next(&mut self) -> Option<Self::Item> {
        // Each item needs 8 bytes: 4 for anchor_sequence + 4 for transaction_payload_checksum
        if self.position + 8 > self.end_position {
            return None;
        }

        // Extract anchor_sequence (u32) from 4 bytes
        let mut anchor_sequence_bytes = [0u8; 4];
        anchor_sequence_bytes.copy_from_slice(&self.payload[self.position..self.position + 4]);
        let anchor_sequence = u32::from_le_bytes(anchor_sequence_bytes);

        // Extract transaction_payload_checksum (u32) from next 4 bytes
        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[self.position + 4..self.position + 8]);
        let transaction_payload_checksum = u32::from_le_bytes(checksum_bytes);

        // Advance position to the next item
        self.position += 8;

        Some(MempoolItem {
            anchor_sequence,
            transaction_payload_checksum,
        })
    }
}

impl PartialEq for RadioMessage {
    fn eq(&self, other: &Self) -> bool {
        // Check message type equality first
        if self.message_type() != other.message_type() {
            return false;
        }

        // Check the specific fields based on the message type
        // Note: sender_node_id is ignored for equality except for RequestEcho messages
        match self.message_type() {
            msg_type if msg_type == MessageType::RequestEcho as u8 => {
                // RequestEcho is an exception - we must check sender_node_id
                self.sender_node_id() == other.sender_node_id()
            }
            msg_type if msg_type == MessageType::Echo as u8 => {
                // Check target_node and link_quality (excluding sender_node_id)

                // Extract target_node from both messages
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_target_node = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_target_node = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_target_node != other_target_node {
                    return false;
                }

                // Extract link_quality from both messages
                if self.length < 10 || other.length < 10 {
                    return false;
                }
                self.payload[9] == other.payload[9]
            }
            msg_type if msg_type == MessageType::EchoResult as u8 => {
                // Check message type and echo result data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare echo result data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            msg_type if msg_type == MessageType::RequestFullBlock as u8 => {
                // Check sequence only (excluding sender_node_id)

                // Extract sequence from both messages
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                self_sequence == other_sequence
            }
            msg_type if msg_type == MessageType::RequestBlockPart as u8 => {
                // Check sequence, payload_checksum, packet_number (excluding sender_node_id)

                // Extract sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_sequence != other_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                if self_checksum != other_checksum {
                    return false;
                }

                // Extract packet_number
                if self.length < 14 || other.length < 14 {
                    return false;
                }
                self.payload[13] == other.payload[13]
            }
            msg_type if msg_type == MessageType::AddBlock as u8 => {
                // Check sequence and payload_checksum (excluding sender_node_id)

                // Extract sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_sequence != other_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                self_checksum == other_checksum
            }
            msg_type if msg_type == MessageType::AddTransaction as u8 => {
                // Check anchor_sequence and payload_checksum (excluding sender_node_id)

                // Extract anchor_sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_anchor_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_anchor_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_anchor_sequence != other_anchor_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                self_checksum == other_checksum
            }
            msg_type if msg_type == MessageType::RequestNewMempoolItem as u8 => {
                // Check message type and mempool data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare mempool data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            msg_type if msg_type == MessageType::Support as u8 => {
                // Check message type and support data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare support data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            _ => {
                // Unknown message type
                false
            }
        }
    }
}

#[derive(Clone)]
pub struct RadioPacket {
    pub data: [u8; RADIO_PACKET_SIZE],
    pub length: usize,
}

impl RadioPacket {
    pub fn message_type(&self) -> u8 {
        self.data[0]
    }

    pub fn sender_node_id(&self) -> u32 {
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.data[1..5]);
        u32::from_le_bytes(node_id_bytes)
    }

    pub fn total_packet_count(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_PACKET_HEADER_SIZE {
                return 0; // Not enough data for packet count
            }
            return self.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE];
        } else {
            return 1; // For other message types, always 1 packet
        }
    }

    pub fn sequence(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8
            && self.message_type() != MessageType::RequestFullBlock as u8
            && self.message_type() != MessageType::Support as u8
            && self.message_type() != MessageType::RequestBlockPart as u8
        {
            return None; // Only AddBlock and AddTransaction messages have a sequence
        }

        // Extract the sequence number from the message payload
        if self.length < 9 {
            return None; // Not enough data for sequence
        }
        let mut sequence_bytes = [0u8; 4];
        sequence_bytes.copy_from_slice(&self.data[5..9]);
        Some(u32::from_le_bytes(sequence_bytes))
    }

    pub(crate) fn payload_checksum(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8
            && self.message_type() != MessageType::RequestFullBlock as u8
            && self.message_type() != MessageType::Support as u8
            && self.message_type() != MessageType::RequestBlockPart as u8
        {
            return None; // Only AddBlock and AddTransaction messages have a sequence
        }

        // Extract the payload checksum from the message payload
        if self.length < 13 {
            return None; // Not enough data for payload checksum
        }
        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.data[9..13]);
        Some(u32::from_le_bytes(checksum_bytes))
    }

    pub fn packet_index(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_PACKET_HEADER_SIZE {
                return 0; // Not enough data for packet index
            }
            return self.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + 1];
        } else {
            return 0; // For other message types, always 0
        }
    }

    pub(crate) fn same_message(&self, other_header: &[u8]) -> bool {
        if self.message_type() != other_header[0] {
            return false;
        }

        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE || other_header.len() < RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE {
                return false;
            }

            if self.data[5..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] == other_header[5..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}

impl Eq for RadioMessage {}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn request_echo_basics_and_eq() {
        let node = 0x0A0B0C0D;
        let m1 = RadioMessage::new_request_echo(node);
        assert_eq!(m1.message_type(), MessageType::RequestEcho as u8);
        assert_eq!(m1.length(), 5);
        assert_eq!(m1.sender_node_id(), node);

        // Same sender -> equal; different sender -> not equal
        let m2 = RadioMessage::new_request_echo(node);
        let m3 = RadioMessage::new_request_echo(123);
        assert_eq!(m1, m2);
        assert_ne!(m1, m3);
    }

    #[test]
    fn echo_fields_and_eq_ignore_sender() {
        let a = RadioMessage::new_echo(100, 200, 7);
        assert_eq!(a.message_type(), MessageType::Echo as u8);
        let (tgt, lq) = a.get_echo_data().unwrap();
        assert_eq!((tgt, lq), (200, 7));

        // Different sender but same target+quality -> equal
        let b = RadioMessage::new_echo(999, 200, 7);
        assert_eq!(a, b);

        // Different target -> not equal
        let c = RadioMessage::new_echo(999, 201, 7);
        assert_ne!(a, c);
    }

    #[test]
    fn echo_result_iterator_and_eq_ignores_sender() {
        let mut er1 = RadioMessage::new_echo_result(42);
        er1.add_echo_result_item(2, 10, 11).unwrap();
        er1.add_echo_result_item(3, 20, 21).unwrap();

        // Iterate and collect
        let items: Vec<(u32, u8, u8)> = er1
            .get_echo_result_data_iterator()
            .unwrap()
            .map(|it| (it.neighbor_node(), it.send_link_quality(), it.receiving_link_quality()))
            .collect();
        assert_eq!(items, vec![(2, 10, 11), (3, 20, 21)]);

        // Same data, different sender -> equal
        let mut er2 = RadioMessage::new_echo_result(7);
        er2.add_echo_result_item(2, 10, 11).unwrap();
        er2.add_echo_result_item(3, 20, 21).unwrap();
        assert_eq!(er1, er2);

        // Different data -> not equal
        let mut er3 = RadioMessage::new_echo_result(7);
        er3.add_echo_result_item(2, 10, 12).unwrap();
        assert_ne!(er1, er3);
    }

    #[test]
    fn mempool_add_iter_and_capacity() {
        let mut m = RadioMessage::new_get_mempool_state(11);
        assert_eq!(m.message_type(), MessageType::RequestNewMempoolItem as u8);

        // Fill up to capacity
        let capacity = (RADIO_PACKET_SIZE - 5) / 8; // items of 8 bytes after 5-byte header
        for i in 0..capacity {
            m.add_mempool_item(i as u32, (i as u32) ^ 0xDEAD_BEEF).unwrap();
        }
        // Next add should fail
        assert!(m.add_mempool_item(999, 999).is_err());

        // Iterate and verify
        let items: Vec<(u32, u32)> = m
            .get_mempool_data_iterator()
            .unwrap()
            .map(|it| (it.anchor_sequence(), it.transaction_payload_checksum()))
            .collect();
        assert_eq!(items.len(), capacity);
        assert_eq!(items[0], (0, 0xDEAD_BEEF));
    }

    #[test]
    fn add_transaction_get_data_and_reply_logic() {
        let payload = &[1, 2, 3, 4, 5];
        let at = RadioMessage::new_add_transaction(7, 1234, 0xABCD_0123, payload);
        assert_eq!(at.message_type(), MessageType::AddTransaction as u8);
        let (anchor, checksum, data) = at.get_add_transaction_data().unwrap();
        assert_eq!(anchor, 1234);
        assert_eq!(checksum, 0xABCD_0123);
        assert_eq!(data, payload);

        // Case 1: Request mempool without matching item -> is_reply_to == true
        let mut req1 = RadioMessage::new_get_mempool_state(9);
        req1.add_mempool_item(1, 2).unwrap();
        assert!(at.is_reply_to(&req1));

        // Case 2: Request mempool includes matching item -> is_reply_to == false
        let mut req2 = RadioMessage::new_get_mempool_state(9);
        req2.add_mempool_item(1234, 0xABCD_0123).unwrap();
        assert!(!at.is_reply_to(&req2));
    }

    #[test]
    fn add_block_fragmentation_and_reassembly() {
        let seq = 0x0102_0304;
        // Payload length spanning multiple packets
        let part = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE; // bytes per packet chunk
        let total_len = part * 3 + 10; // 3 full + 1 partial
        let mut payload_vec = vec![0u8; total_len];
        // Make payload deterministic
        for (i, b) in payload_vec.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        let msg = RadioMessage::new_add_block(5, seq, &payload_vec);

        // Packetization
        let count = msg.get_packet_count();
        assert_eq!(count, 4);

        let mut reassembled = RadioMessage::new();
        for idx in 0..count {
            let p = msg.get_packet(idx).expect("packet must exist");
            // Validate per-packet header/meta
            assert_eq!(p.message_type(), MessageType::AddBlock as u8);
            assert_eq!(p.total_packet_count() as usize, count);
            assert_eq!(p.packet_index() as usize, idx);
            // Feed into reassembler
            reassembled.add_packet(&p);
        }

        // Reassembled message should match type/sequence/checksum and byte length
        assert_eq!(reassembled.message_type(), MessageType::AddBlock as u8);
        assert_eq!(reassembled.sequence().unwrap(), seq);
        assert_eq!(reassembled.length(), RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + total_len);

        // Verify payload bytes match exactly
        assert_eq!(
            &reassembled.payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + total_len],
            &payload_vec[..]
        );

        // Out-of-range packet request returns None
        assert!(msg.get_packet(count).is_none());
    }

    #[test]
    fn new_from_single_packet_roundtrip_simple() {
        let m = RadioMessage::new_request_full_block(21, 0xDEAD_BEEF);
        let p = m.get_packet(0).unwrap();
        // Single packet reconstruction should be equal
        let m2 = RadioMessage::from_single_packet(p);
        assert_eq!(m2, m);
    }

    #[test]
    fn request_block_part_fields_via_eq() {
        let mut a = RadioMessage::new_request_block_part(44, 0xCAFEBABE, 0x1234_5678);
        a.add_packet_index_to_request_block_part(9).unwrap(); // count = 1

        let mut b = RadioMessage::new_request_block_part(9999, 0xCAFEBABE, 0x1234_5678);
        b.add_packet_index_to_request_block_part(9).unwrap(); // count = 1
        // Equality ignores sender for this type, compares seq/checksum and count of requested parts
        assert_eq!(a, b);

        // Different count of packet indices -> not equal under current semantics
        let mut c = RadioMessage::new_request_block_part(44, 0xCAFEBABE, 0x1234_5678);
        c.add_packet_index_to_request_block_part(10).unwrap();
        c.add_packet_index_to_request_block_part(11).unwrap(); // count = 2
        assert_ne!(a, c);
    }

    #[test]
    fn non_chunked_out_of_range_packet_request() {
        let m = RadioMessage::new_request_echo(123);
        // Only index 0 is valid for non-chunked messages
        assert!(m.get_packet(0).is_some());
        assert!(m.get_packet(1).is_none());
    }

    #[test]
    fn chunk_boundary_counts_and_indices() {
        let chunk = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        // Test several boundary sizes up to the maximum
        for n in 1..=core::cmp::min(4, RADIO_MAX_PACKET_COUNT) {
            let total_len = chunk * n; // exact multiple of chunk
            let payload: Vec<u8> = (0..total_len).map(|i| (i % 251) as u8).collect();
            let msg = RadioMessage::new_add_block(77, 0x01020304, &payload);

            assert_eq!(msg.get_packet_count(), n, "packet count should equal chunks for n={}", n);

            for idx in 0..n {
                let p = msg.get_packet(idx).expect("packet must exist");
                assert_eq!(p.total_packet_count() as usize, n);
                assert_eq!(p.packet_index() as usize, idx);
            }

            // Out of range
            assert!(msg.get_packet(n).is_none());
        }
    }

    #[test]
    fn mask_filtering_and_index_mapping() {
        let chunk = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        let n = 5; // total unfiltered packets
        let payload: Vec<u8> = (0..(chunk * n)).map(|i| (i % 251) as u8).collect();

        let mut mask = [false; RADIO_MAX_PACKET_COUNT];
        mask[1] = true;
        mask[2] = true;
        mask[4] = true;

        let msg = RadioMessage::new_add_block_with_packet_list(9, 0xAA55AA55, &payload, mask);
        assert_eq!(msg.get_packet_count(), 3);

        // Filtered index 0 -> actual 1
        let p0 = msg.get_packet(0).expect("filtered packet 0 exists");
        assert_eq!(p0.total_packet_count(), n as u8);
        assert_eq!(p0.packet_index(), 1);

        // Filtered index 1 -> actual 2
        let p1 = msg.get_packet(1).expect("filtered packet 1 exists");
        assert_eq!(p1.packet_index(), 2);

        // Filtered index 2 -> actual 4
        let p2 = msg.get_packet(2).expect("filtered packet 2 exists");
        assert_eq!(p2.packet_index(), 4);

        // Filtered index 3 -> None
        assert!(msg.get_packet(3).is_none());
    }

    #[test]
    fn zero_length_chunked_payload_behaviour() {
        let msg = RadioMessage::new_add_block(5, 0x01020304, &[]);
        // Zero-length payload currently yields 0 packets
        assert_eq!(msg.get_packet_count(), 0);
        assert!(msg.get_packet(0).is_none());
    }

    #[test]
    fn request_block_part_iterator_zero_and_some() {
        // zero indices
        let m0 = RadioMessage::new_request_block_part(1, 0xAABBCCDD, 0x11223344);
        assert_eq!(m0.message_type(), MessageType::RequestBlockPart as u8);
        // count byte is 0, iterator should be empty
        let v0: Vec<u8> = m0.get_request_block_part_iterator().unwrap().map(|it| it.packet_index()).collect();
        assert!(v0.is_empty());

        // one index
        let mut m1 = RadioMessage::new_request_block_part(2, 0xCAFEBABE, 0x55667788);
        m1.add_packet_index_to_request_block_part(9).unwrap();
        let v1: Vec<u8> = m1.get_request_block_part_iterator().unwrap().map(|it| it.packet_index()).collect();
        assert_eq!(v1, vec![9]);

        // multiple indices in order
        let mut mm = RadioMessage::new_request_block_part(3, 0x01020304, 0x99AA_BBCC);
        for i in [1u8, 3, 4, 7, 9] {
            mm.add_packet_index_to_request_block_part(i).unwrap();
        }
        let v: Vec<u8> = mm.get_request_block_part_iterator().unwrap().map(|it| it.packet_index()).collect();
        assert_eq!(v, vec![1, 3, 4, 7, 9]);
    }
}
