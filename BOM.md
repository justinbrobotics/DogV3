# Bill of materials

Quantities below describe the saved four-leg CAD layout and documented code baseline. Hardware labels, wiring, fasteners and print settings still need confirmation on the physical build.

## Printed parts

| ID | Part | Qty | Material | STEP file |
| --- | --- | ---: | --- | --- |
| P01 | Bottom shell | 1 | PLA-CF | [Bottom_Shell.step](printed-parts/Bottom_Shell.step) |
| P02 | Top shell | 1 | PLA-CF | [Top_Shell.step](printed-parts/Top_Shell.step) |
| P03 | Body mounting brackets | 2 | PLA-CF | [Body_Mounting_Bracket.step](printed-parts/Body_Mounting_Bracket.step) |
| P04 | Front head piece | 1 | PLA-CF | [Front_Head.step](printed-parts/Front_Head.step) |
| P05 | Lower electronics tray | 1 | PLA-CF | [Lower_Electronics_Tray.step](printed-parts/Lower_Electronics_Tray.step) |
| P06 | Upper electronics tray | 1 | PLA-CF | [Upper_Electronics_Tray.step](printed-parts/Upper_Electronics_Tray.step) |
| P07 | Tail | 1 | PLA-CF | [Tail.step](printed-parts/Tail.step) |
| P08 | Original shoulder | 2 | TBD | [Shoulder_Original.step](printed-parts/Shoulder_Original.step) |
| P09 | Mirrored shoulder | 2 | TBD | [Shoulder_Mirrored.step](printed-parts/Shoulder_Mirrored.step) |
| P10 | Original LV2 leg link | 2 | TBD | [Leg_Link_LV2_Original.step](printed-parts/Leg_Link_LV2_Original.step) |
| P11 | Mirrored LV2 leg link | 2 | TBD | [Leg_Link_LV2_Mirrored.step](printed-parts/Leg_Link_LV2_Mirrored.step) |
| P12 | Original LV3 leg link | 2 | TBD | [Leg_Link_LV3_Original.step](printed-parts/Leg_Link_LV3_Original.step) |
| P13 | Mirrored LV3 leg link | 2 | TBD | [Leg_Link_LV3_Mirrored.step](printed-parts/Leg_Link_LV3_Mirrored.step) |
| P14 | Original split leg piece, Test1 | 2 | TBD | [Split_Leg_Piece_Test1_Original.step](printed-parts/Split_Leg_Piece_Test1_Original.step) |
| P15 | Mirrored split leg piece, Test1 | 2 | TBD | [Split_Leg_Piece_Test1_Mirrored.step](printed-parts/Split_Leg_Piece_Test1_Mirrored.step) |
| P16 | Original foot, Test2 | 2 | TPU 90A | [Foot_Original.step](printed-parts/Foot_Original.step) |
| P17 | Mirrored foot, Test2 | 2 | TPU 90A | [Foot_Mirrored.step](printed-parts/Foot_Mirrored.step) |

Total: **28 printed instances across 17 designs**. Original and mirrored variants must be printed as listed. PLA-CF applies to the main body/shell group; TPU 90A applies to the feet. Remaining leg materials, layer heights, wall counts, infill, orientation and supports are unconfirmed.

## Purchased parts and fasteners

| ID | Item | Qty / basis | Specification and remaining uncertainty |
| --- | --- | --- | --- |
| H01 | Bus servos | 12 | Current code says Feetech STS3215-C047, 12 V; older list spells ST3215-C047 and advertises 30 kg. Exact physical label/rating is not newly verified. CAD has six original plus six mirrored servo main-case instances; housing/accessory subparts are not additional servos. |
| H02 | Raspberry Pi | 1 | Pi 3B+ is represented and documented; exact installed board revision is unconfirmed. Buy/count a complete board, not its modeled ICs/connectors. |
| H03 | Camera assembly | 1 | Code documents OV5647 5 MP day/night with CSI connection and IR pods. CAD contains a camera assembly with two IR-module subassemblies; exact vendor/kit match is unconfirmed. IR parts are included in this assembly rollup, not separate required purchases. |
| H04 | Speaker | 1 | Code documents an 8 ohm speaker. Exact speaker SKU, dimensions, and rating are unconfirmed. Only one speaker reference remains. |
| H05 | Battery | 1 | Older list specifies a 3S LiPo; capacity, discharge rating, connector, and actual pack dimensions remain TBD. The simplified CAD envelope does not establish suitability or retention. |
| H06 | ESP32 development board | 1 | Owner recalls a WROOM board with mounting holes; older list names an ELEGOO USB-C CP2102 product, while code describes a classic DevKitC clone. Exact variant remains unknown. Mismatched CAD was omitted; tray retained. |
| H07 | Servo bus adapters | 2 | Current code says two Waveshare Bus Servo Adapters. Older list mentions a driver board and links [Bus Servo Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)); exact installed revision/count needs confirmation. Not represented in this CAD layout. |
| H08 | IMU module | 1 | Current code specifies BNO085; older list specifies GY-521/MPU6050. These are different devices. Installed variant remains unconfirmed; not represented in this CAD layout. |
| H09 | Lidar | 1 | Current code specifies RPLIDAR C1. Exact installed unit/accessories remain unconfirmed; no lidar hardware model is included. |
| H10 | USB audio adapter | 1 | Current code specifies a Wonrabai USB sound card. Exact SKU/ports remain TBD; not represented in CAD. |
| H11 | Voltage regulator(s) / UBEC | TBD | Current code describes a 5 V, 7 A UBEC; older list says “various buck converters.” Actual count, models, and wiring are unconfirmed; not represented in CAD. |
| H12 | Robot network router | 1 | Current code describes GL.iNet Opal GL-SFT1200. This is a documented network accessory, not a part of the assembled CAD body. Confirm whether it belongs in the intended physical build. |
| H13 | Servo-battery kill switch | TBD | Current code calls for a physical battery kill switch. Exact installed switch, current rating, and mounting details are unconfirmed. |
| H14 | Wiring, connectors, cables, and power protection | TBD | Code discusses USB/CSI links, servo buses, fuses, TVS devices, and rail capacitors. A complete harness/cut list and final component ratings are not established by CAD. |
| H15 | Foot switches / FSRs, optional | TBD | Code documents foot-input support and optional FSRs; this does not confirm sensors are installed. Exclude unless the chosen physical build uses them. |
| F01 | Brass threaded inserts for lid | 2 inferred from CAD pocket pair | Owner identified **uxcell M5 thread × 4 mm length × 7 mm OD**, knurled heat-set inserts, sold in a 50-piece pack. Two is an inferred lid requirement, not a confirmed installed count. The owner previously glued an insert into a reported 6.75 mm hole; no pocket diameter/depth change was made. |
| F02 | Lid screws | 2 inferred | M5 nominal thread to match the stated inserts; head style and length remain TBD. CAD hole alignment does not verify engagement or clearance. |
| F03 | Other mounting screws, nuts, washers, inserts, and servo accessories | TBD | Complete quantities, thread sizes, lengths, and included servo accessories are not established. Do not infer specifications from CAD filenames or assume everything is included with purchased servos. |

The top-shell insert pockets remain **6.75 mm diameter by 4.1 mm deep**. The owner identified **uxcell M5 × 4 mm length × 7 mm OD** brass inserts. The package corrects hole-center alignment, not printed insert fit. Screw length/head style and other fastener quantities must be measured.

Sources: the saved CAD instance inventory, owner-supplied material/insert identification, and the local CODEBASE hardware documentation. Older hardware notes differ from the current code; the exact ESP32 variant and BNO085-versus-MPU6050 discrepancy remain unresolved. Electronics CAD is illustrative.
