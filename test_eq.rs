// Simple test to verify PartialEq implementation
use moonblokz_radio_lib::RadioMessage;

fn main() {
    println!("Testing RadioMessage PartialEq implementation...");

    // Test request echo equality
    let msg1 = RadioMessage::new_request_echo(123);
    let msg2 = RadioMessage::new_request_echo(123);
    let msg3 = RadioMessage::new_request_echo(456);

    assert_eq!(msg1, msg2);
    assert_ne!(msg1, msg3);
    println!("✓ RequestEcho comparison works correctly");

    // Test echo message equality
    let echo1 = RadioMessage::new_echo(123, 456, 80);
    let echo2 = RadioMessage::new_echo(123, 456, 80);
    let echo3 = RadioMessage::new_echo(123, 456, 90); // Different link quality
    let echo4 = RadioMessage::new_echo(123, 789, 80); // Different target
    let echo5 = RadioMessage::new_echo(999, 456, 80); // Different sender

    assert_eq!(echo1, echo2);
    assert_ne!(echo1, echo3);
    assert_ne!(echo1, echo4);
    assert_ne!(echo1, echo5);
    println!("✓ Echo message comparison works correctly");

    // Test full block request equality
    let block1 = RadioMessage::new_request_full_block(123, 456);
    let block2 = RadioMessage::new_request_full_block(123, 456);
    let block3 = RadioMessage::new_request_full_block(123, 789); // Different sequence
    let block4 = RadioMessage::new_request_full_block(999, 456); // Different sender

    assert_eq!(block1, block2);
    assert_ne!(block1, block3);
    assert_ne!(block1, block4);
    println!("✓ RequestFullBlock comparison works correctly");

    // Test add transaction equality
    let payload = b"test payload";
    let tx1 = RadioMessage::new_add_transaction(123, 456, 789, payload);
    let tx2 = RadioMessage::new_add_transaction(123, 456, 789, payload);
    let tx3 = RadioMessage::new_add_transaction(123, 456, 999, payload); // Different checksum
    let tx4 = RadioMessage::new_add_transaction(123, 999, 789, payload); // Different anchor_sequence

    assert_eq!(tx1, tx2);
    assert_ne!(tx1, tx3);
    assert_ne!(tx1, tx4);
    println!("✓ AddTransaction comparison works correctly");

    // Test different message types are never equal
    assert_ne!(msg1, echo1);
    assert_ne!(echo1, block1);
    assert_ne!(block1, tx1);
    println!("✓ Different message types are correctly identified as not equal");

    println!("\nAll tests passed! ✅ PartialEq implementation is working correctly.");
}
