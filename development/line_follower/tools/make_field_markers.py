"""Reproducible A4 field labels. Actual-size cards; overview is NOT to scale."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.colors import HexColor, black, white
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/pdf/小车场地标记_AB侧方_BC倒库.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(TTFont('CN', 'C:/Windows/Fonts/simhei.ttf'))
c = canvas.Canvas(str(OUT), pagesize=(210*mm,297*mm))
c.setTitle('STM32小车场地标记：AB侧方、BC倒库')
NAVY = HexColor('#19344C'); GRAY=HexColor('#526577')
RED=HexColor('#BA292C'); YELLOW=HexColor('#F1C442')

def text(x,y,s,size=11,color=NAVY,center=False,font='CN'):
    c.setFillColor(color); c.setFont(font,size)
    (c.drawCentredString if center else c.drawString)(x*mm,y*mm,s)

def rule(x,y,w,color=GRAY):
    c.setStrokeColor(color); c.setLineWidth(.6); c.line(x*mm,y*mm,(x+w)*mm,y*mm)

def header(title,sub,page):
    text(16,280,title,18); text(16,270,sub,9,GRAY)
    rule(16,264,178)
    text(16,13,'2026-10-14 | 按当前实物布置制作 | 打印选“实际大小 / 100%”',8,GRAY)
    text(194,13,str(page),9,GRAY,True)

def calibration(y=24):
    rule(16,y,50,black)
    for x in (16,66):
        c.line(x*mm,(y-2)*mm,x*mm,(y+2)*mm)
    text(72,y-1,'此线打印后必须为 50 mm；不要选择“适合页面”。',8,GRAY)

def box(x,y,w,h):
    c.setStrokeColor(GRAY); c.setLineWidth(.6); c.setDash(3,2)
    c.rect(x*mm,y*mm,w*mm,h*mm,fill=0); c.setDash()

def arrow(x1,y1,x2,y2,color=NAVY):
    import math
    c.setStrokeColor(color); c.setLineWidth(1.4)
    c.line(x1*mm,y1*mm,x2*mm,y2*mm)
    a=math.atan2(y2-y1,x2-x1)
    for d in (-.5,.5):
        c.line(x2*mm,y2*mm,(x2-4*math.cos(a+d))*mm,(y2-4*math.sin(a+d))*mm)

header('场地布置与打印说明','AB 侧方停车 / BC 倒车入库 / A、B、C、D 目标点',1)
text(16,251,'先搭场地，再贴标签',12)
for y,s in [(242,'1. 正方形赛道每边 100 cm；黑线宽 1.8 ± 0.2 cm。'),
            (234,'2. AB 内侧中点放 40 × 20 cm 侧方车位，长边沿 AB。'),
            (226,'3. BC 内侧中点放 35 × 30 cm 倒库车位。'),
            (218,'   图中暂按开口宽 30 cm、深 35 cm，正式朝向以现场规定为准。')]: text(16,y,s,10)
# Overview: chosen A->B->C->D counterclockwise. Bay side is vehicle LEFT.
x0,y0,s=45,78,115
c.setStrokeColor(NAVY); c.setLineWidth(4)
c.rect(x0*mm,y0*mm,s*mm,s*mm)
c.setFillColor(YELLOW); c.rect(79.5*mm,78*mm,46*mm,23*mm,stroke=0,fill=1)
c.setFillColor(HexColor('#F0D2D3')); c.rect(119.75*mm,118.25*mm,40.25*mm,34.5*mm,stroke=0,fill=1)
text(102.5,89,'P1 侧方',10,center=True)
text(140,137,'P2 倒库',10,center=True)
text(140,127,'车头朝外 →',8,center=True)
for x,y,t in [(38,69,'A'),(167,69,'B'),(167,198,'C'),(38,198,'D')]:text(x,y,t,20,center=True,font='Helvetica-Bold')
arrow(65,72,100,72); arrow(166,92,166,112); arrow(140,199,105,199); arrow(39,173,39,145)
text(103,160,'内部区域',13,GRAY,True)
text(103,151,'示意图非 1:1',9,GRAY,True)
text(16,60,'注意：原题侧方车位在 DA。本套按你们当前要求改为 AB。',10,RED)
text(16,51,'标签贴在赛道旁或竖立；不要遮挡黑线，也不要铺入灰度探头路径。',9)
text(16,42,'当前固件不能识别文字、颜色或罗马数字；这些卡片先作人工定位。',9)
calibration();c.showPage()

header('车库提示牌','沿虚线裁剪；每张 100 × 100 mm。提示牌不是车位地垫。',2)
for y,code,title,edge,size,color,desc in [
    (151,'P1','侧方停车','AB 内侧中点','车位 40 × 20 cm',YELLOW,'车身平行 AB，完整进入边界'),
    (42,'P2','倒车入库','BC 内侧中点','车位 35 × 30 cm',RED,'倒车驶入，车头朝赛道外侧')]:
    box(55,y,100,100)
    c.setFillColor(color);c.rect(59*mm,(y+78)*mm,92*mm,18*mm,stroke=0,fill=1)
    text(105,y+83,code,25,white if code=='P2' else NAVY,True,'Helvetica-Bold')
    text(105,y+60,title,24,center=True);text(105,y+43,edge,16,center=True)
    text(105,y+28,size,13,center=True);text(105,y+13,desc,10,center=True)
calibration();c.showPage()

header('A / B / C / D 目标点','每张 80 × 80 mm；贴在对应顶点旁，十字仅作人工位置参考。',3)
for x,y,label in [(20,163,'A'),(110,163,'B'),(20,67,'C'),(110,67,'D')]:
    box(x,y,80,80); text(x+40,y+37,label,72,center=True,font='Helvetica-Bold')
    rule(x+33,y+21,14)
    c.line((x+40)*mm,(y+14)*mm,(x+40)*mm,(y+28)*mm)
    text(x+40,y+7,'目标点 / 旁置标签',9,center=True)
text(20,49,'不要把整张白纸压在原来的黑色赛道线上。',10)
calibration();c.showPage()

for page,pairs in [(4,[('I','A'),('III','B')]),(5,[('V','C'),('VII','D')])]:
    header('罗马数字识别卡', '每张 100 × 100 mm；裁剪后只保留白底黑字，供后续视觉识别使用。',page)
    for y,(roman,target) in zip((150,40),pairs):
        box(55,y,100,100)
        # Helvetica Roman characters are intentional, distinct from numbered labels.
        text(105,y+37,roman,100,black,True,'Helvetica-Bold')
        text(160,y+50,'目标 '+target,10)
    calibration();c.showPage()

header('停车调试起点标记','每张 80 × 80 mm；用于记录已测好的车轴起点，不是自动识别触发器。',6)
for y,label,name in [(161,'S1','侧方停车起点'),(66,'S2','倒车入库起点')]:
    box(65,y,80,80)
    text(105,y+61,label,24,center=True,font='Helvetica-Bold')
    arrow(105,y+27,105,y+51)
    rule(83,y+27,44)
    text(105,y+16,name,13,center=True)
    text(105,y+6,'横线对车轴；箭头为车头朝向',8,center=True)
text(16,48,'起点位置须实测后确定：倒库需车轴对准入口中线并留够旋转空间。',9)
text(16,39,'标记可贴车旁作参照；正式发车后仍须由程序自主到达停车起点。',9)
calibration();c.showPage()
c.save()
print(OUT)
