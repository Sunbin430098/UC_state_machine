射环与导航之间的联系

<!-- 边开边决策何时射环    (次要考虑)

背景：导航可能偏向于启动后来回取环以及时更加方便，对于射环时的策略与决策更多为辅助性质

​			同时er射环也可能脱离轨迹控制被迫进行人手操作

联系：结合（无线）手柄操作 

策略：rr结合操作员进行遥控器控制应当更能发挥灵活性,同时台子上方的射环需要实时判断场上形式选择不同的目标柱子进行发射，操作员数量受到限制可以通过遥控器在下位机发送fire指令

流程：

１．枚举设置机器人类型是rr/er(考虑到两个车俯仰结构有无问题)

２．对柱子的特性设置，包括坐标与高度（此处应当预先设置发射装置自身的高度） -->




射环定速发射

背景：机械原因无法完成准确的调速射环功能

目标：确定机构的速度和俯仰角下哪些位置可以射中（计算获得），将这些点组合手柄完成一段又一段的轨迹规划，到位置开射（轨迹规划的状态机需要继续更新功能）

流程：

1.预先输入所有发射区的坐标，结合手柄，通过发送不同命令，生成不同的轨迹，到达不同的发射点

2.结合避障对外的接口，选择目标发射区之后通过避障给出的过程点生成轨迹
    此时该过程应用到RR上的灵活性更高

3．初始位置的准确，需要3D雷达/码盘/激光一起完成定位工作,此时需要做好对接上下位机的通信工作

4.对手柄命令完整应用，应当同时包括实战与调试需要的功能

5.枚举设置机器人类型rr/er(考虑到两个车俯仰结构有无问题),针对不同的车设置不同的射环位姿与发射速度(可能在下位机实现，选择性实践)，对柱子的特性设置，包括坐标与高度（此处应当预先设置发射装置自身的高度）

终极腹黑：先射对面的柱子，给他射懵


检测策略:
    雷达
        A.一直啥也没有　->　空
        B.不是要射的目标柱子，多了一个环　-> 对方
        
    雷达+摄像头
        C.是自己要射的目标柱子，多了一个环 -> 判断颜色

    判断增加东西的逻辑: boxsearch的点增加超过阈值

