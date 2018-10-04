# plan

- **特征匹配确定3D-2D匹配(判定条件)**
	- //LastMap BoW descriptor
	- / 第一帧匹配LastMap
		- Image 搜索 map的匹配 不可信
		- Map 搜索 Image的匹配 更可信
	- / 匹配后RANSAC
		- RANSAC 标准如果定为 p1tFp2 错误的结果可能也会很小(由于相对位姿变化较小？)
		- RANSAC 目前定为筛选较合理的R1, R2后 再计算p1tFp2<th的个数
	- 三角化
	- 初始化
- 3d点的搜素
- 第四个线程:
	- super4pcs(scale 先由descritor确定)
	- ICP 优化3d-3d

- 如何处理Reference Image

- 机械臂
