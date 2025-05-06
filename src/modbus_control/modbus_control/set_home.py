from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore import ModbusSequentialDataBlock
import logging


def main():
    # Enable logging (optional)
    logging.basicConfig()
    log = logging.getLogger()
    log.setLevel(logging.INFO)

    # Create a data block (initial values = 0-99)
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [0]*100),
        co=ModbusSequentialDataBlock(0, [0]*100),
        hr=ModbusSequentialDataBlock(0, [0]*100),
        ir=ModbusSequentialDataBlock(0, [0]*100)
    )

    # Create server context
    context = ModbusServerContext(slaves=store, single=True)

    # Start TCP server on localhost:5020
    print("Starting Modbus TCP server on port 5020...")
    StartTcpServer(context, address=("192.128.1.10", 5020))


if __name__ == '__main__':
    main()