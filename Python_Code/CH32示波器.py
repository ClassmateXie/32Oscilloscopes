import pyqtgraph as pg
import numpy as np
import serial
import time,struct
import pyqtgraph.parametertree as ptree
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets, QT_LIB
from scipy import interpolate 
from PyQt5.QtWidgets import *

N = 512 # 采样点数
k = 3.3/4096 # 比例因子
arr = 400 # 重装载值
div = 72 # 分频因子
Fs = 72000000/arr/div # 采样率
Ts = 1/Fs # 采样周期
COM = 'COM7'

# GUI设计
app = pg.mkQApp()

#============波形显示窗口=====================#
w = pg.GraphicsLayoutWidget()
w.setWindowTitle("CH32示波器-By纯粹")
# 坐标系控件
p = w.addPlot()
p.showGrid(x=True,y=True)
p.setRange(xRange=[-80,80],yRange=[-1,4],padding=0)
p.setLabels(left='电压 / V',bottom="t / ms",title='CH32示波器')
# 时基线和触发门限
inf1 = pg.InfiniteLine(movable=True, angle=90, label='时基={value:0.2f}ms', 
                       labelOpts={'position':0.1, 'color': (200,200,100), 'fill': (200,200,200,50), 'movable': True})
inf2 = pg.InfiniteLine(movable=True, angle=0, pen=(0, 0, 200), bounds = [-20, 20], hoverPen=(0,200,0), label='触发={value:0.2f}V', 
                       labelOpts={'color': (0,200,200), 'movable': True, 'fill': (0, 0, 200, 100)})
inf1.setPos([0,0])
inf2.setPos([0,1])

p.addItem(inf1)
p.addItem(inf2)

curve = p.plot(pen='y') # 曲线控件
#============参数调节窗口=====================#
children=[
	dict(name='采样配置', title='采样配置', type='group', children=[
		dict(name='采样率', type='float', limits=[0.0001, 857.143], value=Fs/1000, units='kHz'),
        dict(name='重装载值', type='int', limits=[2, 65535], value=arr),
        dict(name='分频因子', type='int', limits=[1, 65536], value=div),
		dict(name='采样点数', type='int', limits=[0, 512], value=N),
		dict(name='比例系数', type='float', value=1),
    ]),
	dict(name='虚拟串口', type='str', value=COM),
	dict(name='波特率', type='int', limits=[4800, 20000000], value=1000000),
	dict(name='触发', type='float', value=inf2.getYPos(), units='V'),
	dict(name='时基', type='float', value=inf1.getXPos(), units='ms'),
	dict(name='曲线样式', type='pen', value=pg.mkPen()),
	dict(name='贝塞尔插值', type='bool', value=True),
]
params = ptree.Parameter.create(name='调整参数', type='group', children=children)

def onChanged0(param, val):
	global arr,div,Fs,Ts
	temp = int(72000/val/arr)
	if 1 < temp < 65536:
		params.child('采样配置').child('分频因子').setValue(temp)
	else:
		params.child('采样配置').child('分频因子').setValue(1)
		temp = int(72000/val)
		if 2 < temp < 65536:
			params.child('采样配置').child('重装载值').setValue(temp)	


def onChanged1(param, val):
	global arr,div,Fs,Ts
	if 72000000/val/div > 857143:
		params.child('采样配置').child('重装载值').setValue(arr)
		return
	arr = val
	Fs = 72000000/arr/div
	Ts = 1/Fs
	params.child('采样配置').child('采样率').setValue(Fs/1000)

def onChanged2(param, val):
	global arr,div,Fs,Ts
	if 72000000/val/arr > 857143:
		params.child('采样配置').child('分频因子').setValue(div)
		return
	div = val
	Fs = 72000000/arr/div
	Ts = 1/Fs
	params.child('采样配置').child('采样率').setValue(Fs/1000)

def onChanged3(param, val):
	global N
	N = val
def onChanged4(param, val):
	inf1.setPos([val,0])

def onChanged5(param, val):
	inf2.setPos([0,val])

def onPenChanged(param, pen):
    curve.setPen(pen)

params.child('采样配置').child('采样率').sigValueChanged.connect(onChanged0)
params.child('采样配置').child('重装载值').sigValueChanged.connect(onChanged1)
params.child('采样配置').child('分频因子').sigValueChanged.connect(onChanged2)
params.child('采样配置').child('采样点数').sigValueChanged.connect(onChanged3)

params.child('时基').sigValueChanged.connect(onChanged4)
params.child('触发').sigValueChanged.connect(onChanged5)
params.child('曲线样式').sigValueChanged.connect(onPenChanged)

def On_inf1Changed():
	params.child('时基').setValue(inf1.getXPos())
inf1.sigPositionChanged.connect(On_inf1Changed)

def On_inf2Changed():
	params.child('触发').setValue(inf2.getYPos())
inf2.sigPositionChanged.connect(On_inf2Changed)

pt = ptree.ParameterTree(showHeader=False)
pt.setParameters(params)
# 按钮
StartBtn = QtGui.QPushButton('开始')
StopBtn = QtGui.QPushButton('暂停')
ContinueBtn = QtGui.QPushButton('继续')
EndBtn = QtGui.QPushButton('停止')

run_flag = False
def On_Start():
	global run_flag,ser
	try:
		ser = serial.Serial(params.child('虚拟串口').value(),params.child('波特率').value())
		run_flag = True
		StartBtn.setEnabled(False)
		StopBtn.setEnabled(True)
		ContinueBtn.setEnabled(False)
		EndBtn.setEnabled(True)
	except:
		QtWidgets.QMessageBox(QMessageBox.Warning, '警告', '虚拟串口打开失败').exec_()
	

def On_Continue():
	global run_flag
	run_flag = True
	StartBtn.setEnabled(False)
	StopBtn.setEnabled(True)
	ContinueBtn.setEnabled(False)
	EndBtn.setEnabled(True)

def On_Stop():
	global run_flag
	run_flag = False
	StartBtn.setEnabled(False)
	StopBtn.setEnabled(False)
	ContinueBtn.setEnabled(True)
	EndBtn.setEnabled(False)

def On_End():
	global run_flag,ser
	ser.close()
	run_flag = False
	StartBtn.setEnabled(True)
	StopBtn.setEnabled(False)
	ContinueBtn.setEnabled(False)
	EndBtn.setEnabled(False)

StartBtn.clicked.connect(On_Start)
StopBtn.clicked.connect(On_Stop)
ContinueBtn.clicked.connect(On_Continue)
EndBtn.clicked.connect(On_End)

StartBtn.setEnabled(True)
StopBtn.setEnabled(False)
ContinueBtn.setEnabled(False)
EndBtn.setEnabled(False)

#================主窗口=====================#
win = QtGui.QMainWindow()
win.resize(1000,600)
win.setWindowTitle("CH32示波器-By纯粹")
#================主窗口内添加控件=====================#
cw = QtGui.QWidget()
win.setCentralWidget(cw)

layout = QtGui.QGridLayout()
layout.addWidget(w, 1, 1, 6, 1)
layout.addWidget(pt, 1, 2, 1, 2)
layout.addWidget(StartBtn, 2, 2, 1, 2)
layout.addWidget(StopBtn, 3, 2, 1, 2)
layout.addWidget(ContinueBtn, 4, 2, 1, 2)
layout.addWidget(EndBtn, 5, 2, 1, 2)
cw.setLayout(layout)
win.show()

#================打开虚拟串口=====================#
# ser = serial.Serial(params.child('虚拟串口').value(),params.child('波特率').value())

#=======滑动均值滤波器======#
fps_buf = np.linspace(0,0,100)
fps_buf_ptr = 0
def fps_mean(fps):
	global fps_buf,fps_buf_ptr
	fps_buf[fps_buf_ptr] = fps
	fps_buf_ptr += 1
	if fps_buf_ptr >= 100:
		fps_buf_ptr = 0
	return np.mean(fps_buf)
#==========================#
#=======滑动均值滤波器======#
fc_buf = np.linspace(0,0,20)
fc_buf_ptr = 0
def fc_mean(fc):
	global fc_buf,fc_buf_ptr
	fc_buf[fc_buf_ptr] = fc
	fc_buf_ptr += 1
	if fc_buf_ptr >= 20:
		fc_buf_ptr = 0
	return np.mean(fc_buf)
#==========================#

fps_t0 = cnt = data0 = data1 = 0
start_flag = False
data_buf = np.array([]) # 数据缓存

#更新数据并显示波形
def display_data():
	global cnt,start_flag,data0,data1,data_buf,t0,fps_t0,run_flag
	if run_flag == False:
		return
	if start_flag:
		buf_len = ser.in_waiting
		if buf_len == N*2:
			num = buf_len>>1
			temp = struct.unpack('>%dH'%(num),ser.read(num*2)) # 读取并解析数据

			if temp[0] > 4096: # 12位ADC数据小于4096
				ser.read() # 数据出错,清空接收缓存
				return

			if len(data_buf) < N:
				data_buf = np.append(data_buf,temp)
			else:
				data_buf = np.append(data_buf,temp)
				data_buf = data_buf[-N:]

			output = data_buf*k*params.child('采样配置').child('比例系数').value() # 单位转换 转化为电压
			output_len = len(output)
			t = np.linspace(-500*Ts*output_len,500*Ts*output_len,output_len) # 生成初始时间轴 单位ms
			if params.child('贝塞尔插值').value():
				t_new = np.linspace(-500*Ts*output_len,500*Ts*output_len,output_len*10)
				tck = interpolate.splrep(t, output)
				output_bspline = interpolate.splev(t_new, tck)
				output_bspline_len = len(output_bspline)

				comparator = np.array(output_bspline > inf2.getYPos(),dtype='int8')
				rising = np.where(np.diff(comparator) == 1)[0]
				if len(rising) > 1:
					dt = np.mean(np.diff(rising))*Ts/10
					fc = fc_mean(1/dt)
					start_point = int(output_bspline_len/2+inf1.getXPos()/Ts/100) # 开始触发点索引
					end_point_index = np.where( rising > start_point)[0]
					if len(end_point_index) > 0:
						# 触发成功生成新时间轴 单位ms
						t_new = np.linspace(-100*Ts*(rising[end_point_index[0]]+1)+inf1.getXPos(),
										 100*Ts*(output_bspline_len-rising[end_point_index[0]]-1)+inf1.getXPos(),output_bspline_len)
				else:
					fc = 0

				curve.setData(t_new,output_bspline) # 绘制曲线
			else:
				comparator = np.array(output > inf2.getYPos(),dtype='int8')
				rising = np.where(np.diff(comparator) == 1)[0]
				if len(rising) > 1:
					dt = np.mean(np.diff(rising))*Ts
					fc = fc_mean(1/dt)
					start_point = int(output_len/2+inf1.getXPos()/Ts/1000) # 开始触发点索引
					end_point_index = np.where( rising > start_point)[0]
					if len(end_point_index) > 0:
						# 触发成功生成新时间轴 单位ms
						t = np.linspace(-1000*Ts*(rising[end_point_index[0]]+1)+inf1.getXPos(),
										 1000*Ts*(output_len-rising[end_point_index[0]]-1)+inf1.getXPos(),output_len)
				else:
					fc = 0
				curve.setData(t,output) # 绘制曲线

			fps_t = fps_mean(time.time()-fps_t0) # 帧率均值滤波
			fps_t0 = time.time()

			p.setTitle('CH32示波器 %0.2f fps Fs = %0.3f kHz fc = %0.3f Hz' % (1/fps_t,Fs/1000,fc))
			cnt += num
		if cnt >= N or time.time() - t0 > N*Ts+0.01:# 接收数据满或接收超时
			cnt = 0
			start_flag = False

	else:
		ser.read(ser.in_waiting)
		data_buf = np.array([]) 
		result=ser.write(bytes([0xa5, 0x5a, arr>>8, arr&0xff, div>>8, div&0xff, N>>8, N&0xff, 0xff]))# 发送数据

		t0 = time.time()
		while time.time() - t0 < N*Ts+0.01:# 等待回应
			if ser.in_waiting:
				start_flag = True
				t0 = time.time()
				break


timer = pg.QtCore.QTimer()
timer.timeout.connect(display_data)
timer.start(0)
app.exec_()
