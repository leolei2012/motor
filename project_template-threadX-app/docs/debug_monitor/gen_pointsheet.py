# -*- coding: utf-8 -*-
"""
生成 debug_monitor 的 Modbus 点位表 xlsx（ProbeStation 规范：分组=sheet）。

分组「MCL_OBS」：mcl 全量可观测变量，起始地址 0x2000。
类型三类：uint16(枚举)/uint32/float32。float32 与 uint32 均占 2 寄存器，IEEE754/大端高字在前。

仅用 Python 标准库 zipfile 手写 OpenXML（sharedStrings），无需 openpyxl / exceljs。

字段表与固件 middleware/debug_monitor/src/dm_motor.c 的布局一一对应，改任一需同步。
"""
import zipfile, os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug_monitor-pointsheet.xlsx")

GROUP_NAME = "MCL_OBS"
BASE_ADDR = 0x2000

# 字段表：(别名, 数据类型, 单位, 枚举描述, 寄存器数)
# 枚举用「值=标签;值=标签」字符串（空 = 连续量）
FIELDS = [
    ("运行状态",   "uint16",  "",    "0=IDLE;1=ALIGN;2=RUN;3=FAULT", 1),
    ("故障码",     "uint16",  "",    "0=无;1=过流;2=过压;3=欠压;4=过温;5=堵转;6=门驱", 1),
    ("控制模式",   "uint16",  "",    "0=电流;1=速度;2=位置;3=VF;4=IF;5=预定位", 1),
    ("运行模式",   "uint16",  "",    "0=FOC有感;1=FOC无感;2=BLDC霍尔;3=BLDC无感", 1),
    ("开环阶段",   "uint16",  "",    "0=未开环;1=锁定;2=拖动", 1),
    ("控制周期计数","uint32",  "",    "", 2),
    ("保留",       "uint16",  "",    "", 1),   # 0x07 对齐填充（固件 float 段从 0x08 偶数起）
    ("转速",       "float32", "rpm",  "", 2),
    ("位置",       "float32", "rad",  "", 2),
    ("Iq电流",     "float32", "A",    "", 2),
    ("Id电流",     "float32", "A",    "", 2),
    ("母线电压",   "float32", "V",    "", 2),
    ("母线电流",   "float32", "A",    "", 2),
    ("占空比",     "float32", "",     "", 2),
    ("电机温度",   "float32", "℃",    "", 2),
    ("FET温度",    "float32", "℃",    "", 2),
    ("估计电角度", "float32", "rad",  "", 2),
    ("估计角速度", "float32", "rad/s","", 2),
    ("Iq目标",     "float32", "A",    "", 2),
    ("速度目标",   "float32", "rpm",  "", 2),
    ("开环幅值",   "float32", "pu",   "", 2),
    ("开环角速度", "float32", "rad/s","", 2),
    ("开环相位",   "float32", "rad",  "", 2),
    ("Vα上周期",   "float32", "V",    "", 2),
    ("Vβ上周期",   "float32", "V",    "", 2),
    ("故障电流",   "float32", "A",    "", 2),
    ("故障电压",   "float32", "V",    "", 2),
    ("故障转速",   "float32", "rad/s","", 2),
    ("故障温度",   "float32", "℃",    "", 2),
    ("故障周期计数","uint32",  "",    "", 2),
    ("SMO反电动势α", "float32", "V",    "", 2),
    ("SMO反电动势β", "float32", "V",    "", 2),
    ("SMO角增量w",   "float32", "",     "", 2),
    ("SMO估计电流α", "float32", "A",    "", 2),
]


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def col_letter(idx):  # 0-based
    s = ""
    idx += 1
    while idx:
        idx, r = divmod(idx - 1, 26)
        s = chr(65 + r) + s
    return s


class SharedStrings:
    def __init__(self):
        self._map = {}
        self._list = []

    def add(self, s):
        if s not in self._map:
            self._map[s] = len(self._list)
            self._list.append(s)
        return self._map[s]

    def xml(self):
        items = ''.join(f'<si><t>{esc(s)}</t></si>' for s in self._list)
        return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                f'<sst xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
                f'count="{len(self._list)}" uniqueCount="{len(self._list)}">{items}</sst>')


def write_xlsx():
    ss = SharedStrings()

    # 设备信息 sheet 行
    info_rows = []
    for i, (k, v) in enumerate([("设备名", "motor_app"), ("连接", "RTU:COM6"),
                                ("从站", "1"), ("扫描间隔ms", "200"), ("分组数", "1")], start=1):
        info_rows.append((i, [(1, 's', k), (2, 's', v)]))

    # MCL_OBS 分组 sheet 行
    total_regs = sum(f[4] for f in FIELDS)
    obs_rows = []
    obs_rows.append((1, [(1, 's', '分组'), (2, 's', GROUP_NAME)]))
    obs_rows.append((2, [(1, 's', '从站'), (2, 'n', 1),
                         (3, 's', '功能码'), (4, 'n', 3),
                         (5, 's', '起始地址'), (6, 'n', BASE_ADDR),
                         (7, 's', '数量'), (8, 'n', total_regs)]))
    headers = ['别名', '数据类型', '单位', '系数', '偏移', '枚举', '功能码', '起始地址', '数量']
    obs_rows.append((4, [(i + 1, 's', h) for i, h in enumerate(headers)]))

    r = 5
    addr = BASE_ADDR
    for (alias, dtype, unit, enum, nreg) in FIELDS:
        obs_rows.append((r, [
            (1, 's', alias), (2, 's', dtype), (3, 's', unit),
            (4, 'n', 1), (5, 'n', 0), (6, 's', enum),
            (7, 'n', 3), (8, 'n', addr), (9, 'n', nreg),
        ]))
        addr += nreg
        r += 1

    # 收集字符串
    for _, cells in info_rows + obs_rows:
        for (_, typ, val) in cells:
            if typ == 's':
                ss.add(val)

    def render(rows, max_col):
        body = []
        for rownum, cells in rows:
            body.append(f'<row r="{rownum}">')
            for (col, typ, val) in cells:
                ref = f"{col_letter(col - 1)}{rownum}"
                if typ == 's':
                    body.append(f'<c r="{ref}" t="s"><v>{ss.add(val)}</v></c>')
                else:
                    body.append(f'<c r="{ref}"><v>{val}</v></c>')
            body.append('</row>')
        max_row = max((rn for rn, _ in rows), default=0)
        dim = f"A1:{col_letter(max_col - 1)}{max_row}"
        return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
                'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
                f'<dimension ref="{dim}"/><sheetData>{"".join(body)}</sheetData></worksheet>')

    ct = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
          '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="xml" ContentType="application/xml"/>'
          '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
          '<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
          '<Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
          '<Override PartName="/xl/sharedStrings.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sharedStrings+xml"/>'
          '</Types>')

    root_rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                 '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                 '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
                 '</Relationships>')

    wb_rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
               '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
               '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>'
               '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>'
               '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/sharedStrings" Target="sharedStrings.xml"/>'
               '</Relationships>')

    wb = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
          '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
          'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
          '<sheets>'
          '<sheet sheetId="1" name="设备信息" r:id="rId1"/>'
          '<sheet sheetId="2" name="MCL_OBS" r:id="rId2"/>'
          '</sheets></workbook>')

    entries = {
        '[Content_Types].xml': ct,
        '_rels/.rels': root_rels,
        'xl/workbook.xml': wb,
        'xl/_rels/workbook.xml.rels': wb_rels,
        'xl/sharedStrings.xml': ss.xml(),
        'xl/worksheets/sheet1.xml': render(info_rows, 2),
        'xl/worksheets/sheet2.xml': render(obs_rows, 9),
    }

    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED) as z:
        for name, data in entries.items():
            z.writestr(name, data)

    print("OK pointsheet:", os.path.basename(OUT), os.path.getsize(OUT), "bytes,", len(FIELDS), "fields")


if __name__ == '__main__':
    write_xlsx()
