"""
Dijkstra grid based planning

author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        # x,y 为 coord index, pind 为联合index

    #魔法函数 print 直接调用
    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    # map, 地图边界最值， x-width, y-height
    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict() #dict 为无序， 所以需要每次从openlist pop时，需要查找
    openset[calc_index(nstart, xw, minx, miny)] = nstart   # key is index, value is Node

    while 1:
        # 获取openlist 中cost 最小的 index:Node
        c_id = min(openset, key=lambda o: openset[o].cost)   #得到key, key=func, 比较cost
        # min对字典处理完，无论比较的是key还是value返回的都是key
        current = openset[c_id] # Node
        print("current", current)   # 前面定义了 Node.__str__

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        # 终止条件判断
        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id] # del key

        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            # 是否地图内的结点, 是否障碍物
            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue
            # 是否已经在clost_list（之前已经找过了耳）, 判断的是key
            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set， 如果在open_list, 检查更新cost
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node    #expansion

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def verify_node(node, obmap, minx, miny, maxx, maxy):
    # 检查是否被占用
    if obmap[node.x][node.y]:
        return False
    # 检查Node是否在有效区间内
    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x > maxx:
        return False
    elif node.y > maxy:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))   # meter
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    print("xwidth:", xwidth)
    print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx # meter
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)  # 遍历世界坐标范围，也就是地图， 这里没考虑分辨率
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True   # obstacle inflation
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox = [] # meter
    oy = []

    for i in range(60):
        ox.append(i)    #下边界
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)  #右边界
        oy.append(i)
    for i in range(61):
        ox.append(i)    #上边界
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)  #左边界
        oy.append(i)
    for i in range(40):
        ox.append(20.0)     #左下中间障碍
        oy.append(i)
    for i in range(40):
        ox.append(40.0)     # 右上中间障碍
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")  # k黑色

        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
