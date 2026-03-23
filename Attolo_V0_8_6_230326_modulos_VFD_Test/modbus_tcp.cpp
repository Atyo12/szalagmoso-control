#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

#include "modbus_tcp.h"
#include "hmi.h"
#include "hmi_map.h"

// ------------------------------------------------------------
// Hálózat beállítás
// ------------------------------------------------------------
// Ezt később a saját hálózatodra állítjuk át.
static byte g_mac[] = { 0x02, 0x12, 0x34, 0x56, 0x78, 0x9A };
static IPAddress g_ip(192, 168, 1, 180);
static IPAddress g_dns(192, 168, 1, 1);
static IPAddress g_gateway(192, 168, 1, 1);
static IPAddress g_subnet(255, 255, 255, 0);

EthernetServer ethernetServer(502);
ModbusTCPServer modbusTCPServer;

// ------------------------------------------------------------
// Belső segédek
// ------------------------------------------------------------
static uint16_t clampHzX10(float hz)
{
  if (hz < 0.0f) hz = 0.0f;
  float x10 = hz * 10.0f;
  if (x10 > 65535.0f) x10 = 65535.0f;
  return (uint16_t)(x10 + 0.5f);
}

static void updateModbusOutputsFromStatus()
{
  const HMIStatus& st = hmiGetStatus();

  // --- PLC -> HMI státusz bitek
  modbusTCPServer.coilWrite(HMI_ST_FAULT_ACTIVE,      st.fault_active);
  modbusTCPServer.coilWrite(HMI_ST_RESET_REQUEST,     st.reset_request);

  modbusTCPServer.coilWrite(HMI_ST_HANDLE_MODE,       st.handle_mode);
  modbusTCPServer.coilWrite(HMI_ST_SERVICE_ENABLE,    st.service_enable);

  modbusTCPServer.coilWrite(HMI_ST_WASH_STOP_REQUEST, st.wash_stop_request);

  modbusTCPServer.coilWrite(HMI_ST_RET_RUNNING,       st.return_running);
  modbusTCPServer.coilWrite(HMI_ST_RET_FORCE_EMPTY,   st.return_force_empty);
  modbusTCPServer.coilWrite(HMI_ST_RET_CONTINUOUS,    st.return_continuous);

  modbusTCPServer.coilWrite(HMI_ST_PUSH_ACTIVE,       st.push_active);

  // --- PLC -> HMI numerikus
  modbusTCPServer.holdingRegisterWrite(HMI_HR_OP_MODE,          (uint16_t)st.op_mode);
  modbusTCPServer.holdingRegisterWrite(HMI_HR_STATE,            (uint16_t)st.state);
  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAULT,            (uint16_t)st.fault);
  modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_CMD_X10, clampHzX10(st.return_freq_cmd_hz));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_ACT_X10, clampHzX10(st.return_freq_act_hz));
}

static void applyModbusInputsToHmi()
{
  // --- HMI -> PLC parancs coilok
  hmiSetHandleMode(modbusTCPServer.coilRead(HMI_COIL_HANDLE_MODE));
  hmiSetServiceEnable(modbusTCPServer.coilRead(HMI_COIL_SERVICE_ENABLE));
  hmiSetReturnForceEmpty(modbusTCPServer.coilRead(HMI_COIL_RET_FORCE_EMPTY));
  hmiSetReturnConveyorContinuous(modbusTCPServer.coilRead(HMI_COIL_RET_CONTINUOUS));

  // reset pulse: élre dolgozzuk fel, majd lenullázzuk
  if (modbusTCPServer.coilRead(HMI_COIL_RESET_PULSE)) {
    hmiPulseReset();
    modbusTCPServer.coilWrite(HMI_COIL_RESET_PULSE, 0);
  }

  // --- HMI -> PLC numerikus paraméter
  uint16_t speedX10 = modbusTCPServer.holdingRegisterRead(HMI_HR_RET_SPEED_SET_X10);
  hmiSetReturnConveyorSpeedHz((float)speedX10 / 10.0f);
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void modbusTcpSetup()
{
  Ethernet.begin(g_mac, g_ip, g_dns, g_gateway, g_subnet);
  ethernetServer.begin();

  if (!modbusTCPServer.begin()) {
    Serial.println("MODBUS TCP server start FAILED");
    return;
  }

  // HMI -> PLC coil tartomány
  modbusTCPServer.configureCoils(0, 200);

  // HMI -> PLC + PLC -> HMI holding register tartomány
  modbusTCPServer.configureHoldingRegisters(0, 400);

  Serial.print("MODBUS TCP server ready at IP: ");
  Serial.println(Ethernet.localIP());
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void modbusTcpLoop()
{
  EthernetClient client = ethernetServer.available();

  if (client) {
    modbusTCPServer.accept(client);

    while (client.connected()) {
      modbusTCPServer.poll();

      // előbb a HMI-ből jövő parancsokat vesszük át
      applyModbusInputsToHmi();

      // utána a PLC státuszt kitesszük a térképre
      hmiUpdateStatus();
      updateModbusOutputsFromStatus();

      // fontos: a fő vezérlési ciklus ne akadjon be örökre egy kliens miatt
      if (!client.available()) {
        break;
      }
    }
  }
}