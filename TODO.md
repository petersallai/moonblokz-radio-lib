# MoonBlokz radio library TODO items

## Implement this items now

- Finish embedded example project
- Documentation
  - Review comments by hand (OK: lib.rs, tx_scheduler, echo.rs, link_quality_calculations.rs, mod.rs (both), simulator.rs,rp_lora_sx1262.rs, radio_packet.rs,radio_message.rs,relay_manager.rs)
  - Write a comprehensive README.md

## Implement this functions during blockchain development

- Handle request transaction part
- Handlre request block with sequence and hash
- Handle gathering mode

## Roadmap items

- Moving node handling is not implemented (maybe it is ok, but needs testing)
- Analyze the effect to refactor RadioMessage and RadioPacket to more idimotic enum form (pro: type-safe structure, easier getters, con: more complex serialization, deserializaton logic, complex structs/enums, more complex message size limit calculations)
