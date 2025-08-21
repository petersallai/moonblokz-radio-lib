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
    RequestNewMempoolItem = 0x08,
    Support = 0x09,
}

#[derive(Debug)]
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
            payload[0..13].copy_from_slice(&packet.data[0..13]);
            // Skip the 2 packet meta bytes at positions 13 and 14 in the packet
            payload[13..13 + (packet.length - 15)].copy_from_slice(&packet.data[15..packet.length]);

            RadioMessage {
                payload,
                // 13 bytes header + actual payload (packet.length - 15)
                length: 13 + (packet.length - 15),
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
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
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
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);

        RadioMessage { payload, length: 9 }
    }

    pub fn new_echo_result(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo results
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::EchoResult as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage { payload, length: 5 }
    }

    pub(crate) fn add_echo_result_item(&mut self, neighbor_node: u32, send_link_quality: u8, receive_link_quality: u8) -> Result<(), ()> {
        // Add an echo result item to the message payload
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

    pub fn new_request_block_part(node_id: u32, sequence: u32, payload_checksum: u32, packet_number: u8) -> Self {
        // Create a new RadioMessage with a specific message type for block part requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestBlockPart as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
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
        full_payload[0] = MessageType::AddBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
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
        payload[0] = MessageType::RequestNewMempoolItem as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage { payload, length: 5 }
    }

    pub fn add_mempool_item(&mut self, anchor_sequence: u32, payload_checksum: u32) -> Result<(), ()> {
        // Add a mempool item to the message payload
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
        full_payload[13..13 + signature.len()].copy_from_slice(signature);

        RadioMessage {
            payload: full_payload,
            length: 13 + signature.len(),
        }
    }

    pub fn message_type(&self) -> u8 {
        if self.length == 0 {
            return 0; // Default to 0 if message has no logical length
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

#[derive(Clone)]
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
        let csum = 0xA1B2_C3D4;
        // Payload length spanning multiple packets
        let part = RADIO_PACKET_SIZE - 15; // bytes per packet chunk
        let total_len = part * 3 + 10; // 3 full + 1 partial
        let mut payload_vec = vec![0u8; total_len];
        // Make payload deterministic
        for (i, b) in payload_vec.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        let msg = RadioMessage::new_add_block(5, seq, csum, &payload_vec);

        // Packetization
        let count = msg.get_packet_count();
        assert_eq!(count, 4);

        let mut reassembled = RadioMessage::new_empty_message();
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
        assert_eq!(reassembled.length(), 13 + total_len);

        // Verify payload bytes match exactly
        assert_eq!(&reassembled.payload[13..13 + total_len], &payload_vec[..]);

        // Out-of-range packet request returns None
        assert!(msg.get_packet(count).is_none());
    }

    #[test]
    fn new_from_single_packet_roundtrip_simple() {
        let m = RadioMessage::new_request_full_block(21, 0xDEAD_BEEF);
        let p = m.get_packet(0).unwrap();
        // Single packet reconstruction should be equal
        let m2 = RadioMessage::new_from_single_packet(p);
        assert_eq!(m2, m);
    }

    #[test]
    fn request_block_part_fields_via_eq() {
        let a = RadioMessage::new_request_block_part(44, 0xCAFEBABE, 0x1234_5678, 9);
        let b = RadioMessage::new_request_block_part(9999, 0xCAFEBABE, 0x1234_5678, 9);
        // Equality ignores sender for this type, compares seq/checksum/part
        assert_eq!(a, b);
        // Different packet number -> not equal
        let c = RadioMessage::new_request_block_part(44, 0xCAFEBABE, 0x1234_5678, 10);
        assert_ne!(a, c);
    }
}
