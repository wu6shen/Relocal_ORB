# plan

- **特征匹配确定3D-2D匹配(判定条件)**
	- //LastMap BoW descriptor
	- / 第一帧匹配LastMap
		- //暴力>knn>>BoW
		- *LastMap 的 Descriptor*
	- // 匹配后RANSAC
		- (10-4) //RANSAC 标准如果定为 p1tFp2 错误的结果可能也会很小(由于相对位姿变化较小？)
		- (10-4) //RANSAC 目前定为筛选较合理的R1, R2后 再计算p1tFp2<th的个数
		- (10-4) //两帧匹配点RANSAC数量太少,只能单帧所有点RANSAC,利用F确定是否合理
		- (10-4) //单帧RANSAC失配率较高，利用初始(或者RANSAC成功的)帧做筛选
		- (10-5) //初始Knn匹配，投影搜索匹配第二帧 最后验证F
	- // (10-5)三角化
	- /初始化
		- (10-5) //合理的判断不太好
		- (10-7) //初始化后失败主要原因是：bundle调整把lastMap的点拽跑、导致Map的质量低，之后会跟丢
		- (10-7) /初始化一段时间fix掉lastMap的点，不允许bundle做优化-------开学了==
- 3d点的搜素
- 第四个线程:
	- super4pcs(scale 先由descritor确定)
	- ICP 优化3d-3d
- 如何处理Reference Image

- 机械臂
