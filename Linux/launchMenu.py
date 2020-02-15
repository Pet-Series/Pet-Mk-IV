#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
import os
menuPath = "/home/pi/ros_ws/src/Pet-Mk-IV/pet_mk_iv_mission_control/nodes"
menuItems = list()

def get_menu_items(filePath):
    Items = list()
    for file in os.listdir(filePath):
        if file.endswith(".py"):
            Items.append(os.path.basename(file))
    return Items

menuItems = get_menu_items(menuPath)
#print(menuItem)
#print(menuItem[0])
#print(menuItem[1])
Row = 0
for menuRow in menuItems:
    Row = Row +1
    print(str(Row) + ") " + menuRow)