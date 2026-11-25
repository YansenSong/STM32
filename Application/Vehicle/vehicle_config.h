#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "stm32f4xx_hal.h"

/* ================= Board resources =================
 * MCU: STM32F407ZGT6
 * UART command port: USART1, PB6=TX, PB7=RX, 115200-8N1
 * CAN1: PA11=RX, PA12=TX, used for EPS 0x314
 * CAN2: PB12=RX, PB13=TX, used for RT49 0x301 and SEB 0x7B9
 */

#define VEH_UART1_BAUDRATE              115200u

/* CAN baud selection.
 * APB1 CAN clock is 42 MHz when SystemClock_Config() in main.c is used.
 * 500K: prescaler 6, total 14 TQ = 42M / 6 / 14 = 500K
 * 250K: prescaler 12, total 14 TQ = 42M / 12 / 14 = 250K
 */
#define VEH_CAN1_EPS_BAUD_250K          1u      /* 1: CAN1 EPS 250K; 0: CAN1 EPS 500K */
#define VEH_CAN2_DRIVE_BRAKE_500K       1u

/* CAN IDs */
#define VEH_CAN_ID_EPS_CMD              0x314u
#define VEH_CAN_ID_RT49_CMD             0x301u
#define VEH_CAN_ID_SEB_REQ              0x7B9u

/* UART command protocol */
#define VEH_CMD_LEN                     26u
#define VEH_CMD_HEADER_LEN              5u
#define VEH_CMD_VERSION_VALUE           0x01u

#define VEH_CMD_VERSION                 5u
#define VEH_CMD_MODE                    6u
#define VEH_CMD_FLAGS                   7u
#define VEH_CMD_ENABLE                  8u
#define VEH_CMD_V_RAW                   9u
#define VEH_CMD_EPS_RAW                 13u
#define VEH_CMD_SEB_MODE                17u
#define VEH_CMD_SEB_VALUE               18u
#define VEH_CMD_LIGHT                   20u
#define VEH_CMD_COUNTER                 21u
#define VEH_CMD_CHECKSUM                22u

/* Mode */
#define VEH_MODE_DISABLE                0u
#define VEH_MODE_AUTO                   1u
#define VEH_MODE_DEBUG                  2u

/* flags byte */
#define VEH_FLAG_ESTOP                  0x01u
#define VEH_FLAG_EPS_CENTER             0x02u
#define VEH_FLAG_SEB_AUTOBRAKE          0x04u
#define VEH_FLAG_SEB_ALIGN              0x08u

/* enable byte */
#define VEH_ENABLE_RT49                 0x01u
#define VEH_ENABLE_EPS                  0x02u
#define VEH_ENABLE_SEB                  0x04u

/* SEB mode */
#define VEH_SEB_MODE_NONE               0u
#define VEH_SEB_MODE_PRESSURE           1u
#define VEH_SEB_MODE_STROKE             2u
#define VEH_SEB_MODE_ALIGNMENT          3u
#define VEH_SEB_MODE_AUTOBRAKE          4u

/* Control parameters */
#define VEH_CONTROL_PERIOD_MS           20u
#define VEH_CMD_TIMEOUT_MS              500u

/* Initial calibration parameter. Adjust after real vehicle speed calibration. */
#define VEH_RT49_RPM_PER_MPS            3000.0f
#define VEH_RT49_RPM_LIMIT              6000
#define VEH_EPS_ANGLE_LIMIT_DEG         462.0f
#define VEH_SEB_ESTOP_PRESSURE_MPA      4.0f

/* SEB checksum and rolling counter. Your test showed checksum/rollcnt may be unnecessary.
 * Keep 0 for first debugging. Set to 1 only if SEB requires it.
 */
#define VEH_SEB_USE_ROLLCNT_CHKSUM      0u




/* ================= Ethernet upper-computer command input =================
 * Hardware: LAN8720 RMII Ethernet interface on board RJ45.
 * Protocol: PC/path-planning host sends the same 26-byte cmd__ frame as UDP payload.
 * STM32 static IP: 192.168.5.50, UDP local port: 5000.
 * Priority: Ethernet and USART1 are both upper-computer command sources;
 *           remote controller still overrides them before CAN output.
 */
#define VEH_ETH_ENABLE                 1u
#define VEH_ETH_UDP_PORT               5000u
#define VEH_ETH_ACCEPT_ANY_UDP_PORT    0u

#define VEH_ETH_LOCAL_IP0              192u
#define VEH_ETH_LOCAL_IP1              168u
#define VEH_ETH_LOCAL_IP2              5u
#define VEH_ETH_LOCAL_IP3              50u

/* Status UDP is unicast on the local /24; no gateway routing in this stack. */
#define VEH_ETH_NETMASK0               255u
#define VEH_ETH_NETMASK1               255u
#define VEH_ETH_NETMASK2               255u
#define VEH_ETH_NETMASK3               0u
#define VEH_STA_HOST_IP0               192u
#define VEH_STA_HOST_IP1               168u
#define VEH_STA_HOST_IP2               5u
#define VEH_STA_HOST_IP3               11u
#define VEH_STA_UDP_PORT               5001u
#define VEH_STA_PERIOD_MS              20u
#define VEH_STA_FEEDBACK_TIMEOUT_MS    200u
#define VEH_STA_ARP_RETRY_MS            1000u
#define VEH_STA_ARP_CACHE_MS            10000u

/* Real CAN feedback, based on the supplied RT49/EPS/SEB protocols.
 * RT49 command 0x301 corresponds to status 0x401; change if node is changed.
 * EPS default follows EXISTING TX: bytes[1:2] are Big-Endian, matrix motorla314.
 * The matrix uses Motorola LSB start-bit positions, not DBC MSB start bits.
 * Set 1 for Intel314 (bytes[1:2] Little-Endian). Never autodetect endian.
 */
#define VEH_CAN_ID_RT49_STATUS          0x401u
#define VEH_CAN_ID_EPS_STATUS           0x18Fu
#define VEH_CAN_ID_EPS_FAULT            0x700u
#define VEH_CAN_ID_SEB_STATUS           0x722u
#define VEH_CAN_ID_SEB_FAULT            0x731u
#define VEH_EPS_RX_INTEL                0u
/* Diagnostic only: accept all standard CAN1 data IDs, keep known parser strict. */
#define VEH_EPS_RX_DIAGNOSTIC           1u
/* CAN1 timing A/B trial: keep250kbps/87.5% sample, widen SJW1 ->2TQ.
 * Set1 to restore original timing. CAN2 is unchanged. */
#define VEH_CAN1_SJW_TQ                 2u
#if (VEH_CAN1_SJW_TQ < 1u) || (VEH_CAN1_SJW_TQ > 2u)
#error CAN1 SJW must be1 or2; fixed BS2 is2TQ
#endif
#define VEH_CAN_RX_MAX_PER_BUS          4u
#define VEH_STA_ERROR_TIMEOUT_MS        500u

/* Keep the legacy speed mapping until the drivetrain is calibrated. */
#define VEH_FEEDBACK_USE_LEGACY_STANDARD 1u
#define VEH_FEEDBACK_SPEED_CALIBRATED   0u
/* 0 unconfigured, 1 measured RPM/(m/s), 2 wheel geometry.
 * DOCX gives radius0.200m but does NOT give current vehicle gear ratio.
 * Legacy UNUSED Core/upper_data.c ratio24.068 was paired with radius0.105m
 * and wheelbase1.205m; confirm applicability before using it here. */
#define VEH_FEEDBACK_SPEED_METHOD       1u
#define VEH_FEEDBACK_WHEEL_RADIUS_M     0.200f
#define VEH_FEEDBACK_GEAR_RATIO         0.0f
#define VEH_FEEDBACK_RPM_PER_MPS        VEH_RT49_RPM_PER_MPS
#define VEH_FEEDBACK_SPEED_SIGN         1
#define VEH_FEEDBACK_STEER_CALIBRATED   1u
/* Bench-verified mapping shared by the command and feedback paths. */
#define VEH_STEERING_WHEEL_PER_EPS_DEG  0.1f
#define VEH_STEERING_WHEEL_OFFSET_DEG   0.0f

#if (VEH_EPS_RX_INTEL > 1u)
#error EPS feedback byte order must be 0 (Motorola314) or 1 (Intel314)
#endif

#define VEH_ETH_MAC0                   0x02u
#define VEH_ETH_MAC1                   0x00u
#define VEH_ETH_MAC2                   0x00u
#define VEH_ETH_MAC3                   0x00u
#define VEH_ETH_MAC4                   0x00u
#define VEH_ETH_MAC5                   0x50u

/* ================= Remote controller input =================
 * Hardware wiring: receiver SBUS -> CN5 pin29/SBUS_RD -> onboard inverter
 *                  -> UART5_RX/PD2. Receiver +5V is CN5 pin28 and signal GND
 *                  is CN5 pin16/17/18. SBUS_TD/UART5_TX is not used.
 * Native SBUS format: 100000 baud, 8E2, 25-byte frame.
 */
#define VEH_REMOTE_ENABLE                  1u
#define VEH_REMOTE_UART5_BAUDRATE          100000u
#define VEH_REMOTE_TIMEOUT_MS              300u

/* Remote channel mapping. Channel numbers are 1-based.
 * CH2 is the normal speed command. CH5 remains the safety arm/takeover switch.
 * Pull CH6 low to override CH2 with a fixed +200rpm test command.
 */
#define VEH_REMOTE_SPEED_CH                2u      /* CH2 up: forward, down: reverse */
#define VEH_REMOTE_ARM_CH                  5u      /* CH5: master arm switch */
#define VEH_REMOTE_FIXED_SPEED_CH          6u      /* CH6 low: fixed-speed test mode */

#define VEH_REMOTE_CENTER_US               1500u
#define VEH_REMOTE_SPEED_START_THRESHOLD_US 80u
#define VEH_REMOTE_ARM_THRESHOLD_US        1700u
#define VEH_REMOTE_FIXED_SPEED_THRESHOLD_US 1300u
#define VEH_REMOTE_FIXED_SPEED_RPM         200

/* CH2 speed mapping:
 *   1420..1580us -> 0rpm
 *   above 1580us -> +100rpm, rising linearly to +400rpm at 2000us
 *   below 1420us -> -100rpm, falling linearly to -400rpm at 1000us
 * This deliberately avoids 1..99rpm where the motor may fail to start.
 */
#define VEH_REMOTE_MIN_MOTOR_RPM           100
#define VEH_REMOTE_MAX_MOTOR_RPM           400

/* Set to 1 only if the real motor direction is opposite to the stick direction. */
#define VEH_REMOTE_SPEED_REVERSE           0u
#define VEH_REMOTE_ALLOW_REVERSE           1u

/* Only healthy CH5-armed input initiates remote takeover.
 * Healthy CH5-off releases takeover to UDP/USART1. After takeover, loss/failsafe
 * keeps the existing latched safe-stop behavior until healthy CH5-off arrives.
 */
#define VEH_REMOTE_OVERRIDE_AFTER_FIRST_FRAME 0u

/* 1: actually send CAN frames; 0: only generate dbg_xxx_tx_data for checking. */
#define VEH_REAL_CAN_TX_ENABLE          1u

#endif
