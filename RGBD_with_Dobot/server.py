from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.server import StartTcpServer
import logging
import time
from threading import Thread

# 로그 설정
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# 레지스터 초기값 설정 (100 ~ 105번)
initial_values = [100, 100, 100, 100, 100, 100]

# 데이터 블록 생성 (100번부터 105번까지 레지스터)
store = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(100, initial_values),  # Holding Registers
)

# 서버 컨텍스트 생성, 유니트 ID 255
context = ModbusServerContext(slaves={255: store}, single=False)

# 서버 식별 정보 (선택 사항)
identity = ModbusDeviceIdentification()
identity.VendorName = "Custom Modbus Server"
identity.ProductCode = "PM11"
identity.VendorUrl = "LEMON KIM"
identity.ProductName = "Modbus Server"
identity.ModelName = "LEMON Model"
identity.MajorMinorRevision = "1.7"


# 레지스터 값 출력 함수
def print_register_values():
    while True:
        time.sleep(1)  # 1초 대기
        values = context[255].getValues(
            3, 99, count=6
        )  # Holding Register(3), 주소 100부터 6개 값
        print(f"Registers 100-105 values: {values}")


# 서버 시작 함수
def run_server():
    StartTcpServer(context, identity=identity, address=("127.0.0.1", 502))


if __name__ == "__main__":
    # 레지스터 출력 쓰레드 시작
    print_thread = Thread(target=print_register_values)
    print_thread.start()

    # 서버 시작
    run_server()
