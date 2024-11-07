from pymodbus.client import ModbusTcpClient
import time
import DobotEDU

home = [210,0,0]
pick1 = [143,-241,-55]
pick2 = [104,-241,-55]
pick3 = [63,-241, -55]

new = [145,259,-46]
new2 = [143,216,-46]


j1_joint = 64

port = 'COM4'  

device=DobotEDU.dobot_magician
device.connect_dobot(port)

def getpos():
    p1=device.get_pose(port)
    print(p1)

def grip():
    device.set_endeffector_gripper(port, enable=True,on=True)

def ungrip():
    device.set_endeffector_gripper(port, enable=False,on=False)

def job1():

    device.set_ptpcmd(port,ptp_mode=1,x=217,y=52,z=40,r=13)
    device.set_ptpcmd(port,ptp_mode=2,x=217,y=52,z=0,r=13)
    ungrip()
    device.set_ptpcmd(port,ptp_mode=2,x=217,y=52,z=40,r=13)
    getpos()
    device.disconnect_dobot(port)

def set_sig(num1,num2):
    #client = ModbusTcpClient('127.0.0.1', port=502)
    connection = client.connect()
    if connection:
        client.write_register(num1,num2,255)
        #client.close()
        

def get_sig(num1):
    #client = ModbusTcpClient('127.0.0.1', port=502)
    connection = client.connect()
    if connection:
        read1= client.read_holding_registers(num1, 1, 255)
        print(read1.registers[0])
        return read1.registers[0]
    

def reset_sig():
    # Modbus TCP 클라이언트 생성
    #client = ModbusTcpClient('127.0.0.1', port=502)
    # 서버에 접속
    connection = client.connect()
    if connection:
        # Holding Register 값을 읽기 
        # register_1 = client.read_holding_registers(0, 1,255)
        client.write_register(0,0,255)#dobot 1번축 값
        client.write_register(1,0,255)#dobot 2번축 값
        client.write_register(2,0,255)#dobot 3번축 값
        client.write_register(3,0,255)#dobot 4번축 값
        
        client.write_register(4,208,255)
        client.write_register(5,0,255)
        client.write_register(6,268,255)
        # 연결 종료
        #client.close()
    else:
        print("Unable to connect to the Modbus server.")
'''
# 클라이언트 코드 실행
if __name__ == "__main__":
    read_coils_and_registers()
'''

client = ModbusTcpClient('192.168.26.32', port=502)
reset_sig()
def work1():
    device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)

    device.set_ptpcmd(port,ptp_mode=1,x=pick1[0],y=pick1[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick1[0],y=pick1[1],z=pick1[2],r=0)
    grip()
    device.set_ptpcmd(port,ptp_mode=2,x=pick1[0],y=pick1[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=1,x=pick1[0],y=-pick1[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick1[0],y=-pick1[1],z=-10,r=0)
    ungrip()
    device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)


def work2():

    device.set_ptpcmd(port,ptp_mode=1,x=pick2[0],y=pick2[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick2[0],y=pick2[1],z=pick2[2],r=0)
    grip()
    device.set_ptpcmd(port,ptp_mode=2,x=pick2[0],y=pick2[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=1,x=pick2[0],y=-pick2[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick2[0],y=-pick2[1],z=-10,r=0)
    ungrip()
    device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)


def work3():
    device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)

    device.set_ptpcmd(port,ptp_mode=1,x=pick3[0],y=pick3[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick3[0],y=pick3[1],z=pick3[2],r=0)
    grip()
    device.set_ptpcmd(port,ptp_mode=2,x=pick3[0],y=pick3[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=1,x=pick3[0],y=-pick3[1],z=60,r=0)
    device.set_ptpcmd(port,ptp_mode=2,x=pick3[0],y=-pick3[1],z=-10,r=0)
    ungrip()
    device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)


device.set_homecmd(port)

while (1):
    R=get_sig(100) #10번 작업 있는지 체크
    G=get_sig(101) #10번 작업 있는지 체크
    B=get_sig(102) #10번 작업 있는지 체크
    time.sleep(1)
    if R:
        work1()
    elif G:
        work2()
    elif B:
        work3()
    else:
        ungrip()
        device.set_ptpcmd(port,ptp_mode=1,x=home[0],y=home[1],z=home[2],r=0)
        