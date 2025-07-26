#MoonBlokz radio library

##Description of the algorithms

There are nodes on the network that communicate using only broadcast radio signals. The majority of the nodes have a fixed position and are working continuously. There are some moving nodes, but they are not the majority of the network. The network is ad-hoc, and the nodes are independent. There is no central authority available to create a structured network (such as partitions).
The nodes build a blockchain and communicate using the following messages:
add_block: propagate a new block in the network.
request_block: requesting a block by sequence number from the network. Nodes use this function if they receive a block whose parent is missing. request_block can also be used to request only one packet from a previous  add_block message. Nodes also query for new blocks (with the last known sequence+1), if no network activity is detected for a long time. 
add_transaction: propagate a new transaction in the network
mempool_state: propagate the hashes of top transactions of a mempool. Nodes use this function to check if new items are available from their network. Nodes periodically (with a low frequency) use this message if they do not have enough transactions to create a full block. If a node knows other top transactions, it replies with an add_transaction message. If multiple transactions are missing, the node randomly chooses one from them.
request_transaction: Request a transaction with a given hash. Nodes use this message if they see a mempool_state message with an unknown transaction. If a node knows this transaction, it replies with an add_transaction message. request_transaction can also be used to request only one packet of a previous add_transaction message.
support: The blockchain uses a deterministic algorithm to choose the creator of the next block. If that node cannot create the block, another node can take its place, but the majority of the network must approve it with a support message.
request_echo: request an echo message from all nodes that directly receive this message.
echo: Reply for request_echo with the received request's signal strength indicator and signal noise ratio.

The messages can be packetized to match the maximum packet size of the used radio technology (current LoRa). All messages have a standard header that includes the sender node's identifier and a payload checksum.

All nodes use periodic request_echo and echo messages to map the network. The frequency of request_echo calls is based on the number of echo responses during the previous cycle. With more nodes in the neighborhood, the request_echo calls are less frequent to preserve bandwidth. Based on the responses, all nodes build a matrix (connection_matrix) to store the signal strength between all of the neighbors. On the rows and columns, there is a list of all neighbors (including the actual node), and in the cells, there are signal strength values from 0 to 127 sent by the node in the row and received by the node in the column. 
The connection matrix has a predefined maximum size. If there are more neighbors than that, only the top N neighbors are stored based on RSSI.
Every row in the connection item matrix has a timestamp for the last echo request. If echo requests arrive too frequently, the logic can ignore it. If a valid echo_request arrives, the logic adds a dirty flag to all values in that row. If an echo arrives, this dirty flag is removed. If a new valid echo_request arrives from the same node, all values with a dirty flag are zeroed. With this logic, one period can be skipped, but if there are no answers for echo_request for two continuous periods, the connection will be deleted.

When a node receives a message, it uses an adaptive algorithm to decide whether to answer/relay it or not.
- Nodes always answer for request_echo messages.
- Nodes never relay echo messages.
- If a request_block message arrives and the node has the requested block, it checks the connection_matrix and makes a list based on connection strength to the requesting node. Next, the node waits for a specified amount of time, based on its position in the list. If many neighbor nodes have better connections, it waits more. If a reply does not arrive during this waiting period, the node sends the reply.
- If a request_block message arrives and the node does not have the requested block, it also waits for a time based on the connection matrix and relays the request block message if nobody relays the message or sends an answer for it later.
- If an add_block message is received and the node does not have this block, it also calculates a time based on connection scores to all neighbor nodes. It calculates how many neighbors are likely not received the message and which nodes can effectively relay the message to reach all nodes. If probably not all nodes received the message (based on the connection_matrix and the relayed messages, a neighbor node probably received the message if it has a connection above a threshold value to any nodes that sent this message) when the waiting time passes, the node relays the message.
- For add_transactions, the same logic applies that we used for add_blocks.
- For mempool_state messages, first, the node decides if it has transactions to send. If there are transactions (these are missing transactions from the requestor side), it randomly chooses one of them and also calculates a waiting time based on the connection matrix. If no other nodes send the selected transaction during this waiting period, the node sends an add_transaction message when the waiting time passes.
- For request_transactions, we use the same logic that we used for request_blocks.
- For support messages, we use the same logic that we used for add_blocks and add_transactions.
 
 To avoid collisions, all waiting time calculations use a small random factor, and all nodes use channel activity detection to delay transmission if the channel is not free. They also delay transactions if a node already started to send a multi-packet message.
Only fixed nodes are added to the connection_matrix. If a fixed node receives a message from a moving node, it calculates waiting times based on coverage to other neighbor nodes. If a moving node receives a message, it uses a random function with a constant average to calculate the waiting time. This waiting time is typically longer than waiting times for fixed nodes. 
There is no TTL logic required for any messages. A message (support, add_block, add_transaction) is only relayed if the node does not already have this message. If a message is already distributed in the network, the relaying stops quickly.
The network does not use any response messages for successful transmissions. If a node does not receive an add_block message, it will notify it one block later because the parent of the next block will be missing, and the node will send a request_block message to get it. If no reply arrives for a given time, the node will resend the request_block message.
The blockchain network is delay-tolerant by design, and nodes can handle out-of-order messages; therefore, there is no need to prioritize between message types. 
Nodes utilize multiple parameters for fine-tuning the algorithm. There are a few parameters that are defined at compile time (such as the size of the connection matrix). Still, most of the parameters come from the chain's configuration and are distributed by the genesis block.
If the nodes are separated into groups, divergent chains can be formed. The consensus algorithm limits it. Sooner or later, all sub-groups reach a state where the deterministic chain creator selection logic selects a node that is not in that subgroup. In that case, an approval process begins, and it will likely only succeed in the sub-group that has more than half of the active nodes. If two sub-groups connect, all nodes query all the block variants (through the request_block logic), and creators always select the heaviest chain variants (the weight of the block is 1, except the evidence block for support messages, where the weight equals to the number of the support messages).





