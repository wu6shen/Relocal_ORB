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
	- // 初始化
		- (10-5) //合理的判断不太好
		- (10-7) //初始化后失败主要原因是：bundle调整把lastMap的点拽跑、导致Map的质量低，之后会跟丢
		- (10-8) //第一帧超过80个点
		- (10-8) //第二帧计算的F与第一帧超过100个点、两帧有lastMap对应点重投影错误<10、至少250个点、点的距离足够长
		- (10-8) /初始化一段时间fix掉lastMap的点(3d点的个数、关键帧的个数?)，不允许bundle做优化-------开学了==
		- (10-8) /不如原初始鲁棒;;;;可能这样来做匹配之后再在ICP下RANSAC效果更好 但是ICP就需要比较高质量的点、导致初始化时间可能更长
- 3d点的搜素
	- (10-9) 点的收敛 考虑所有参与优化关键帧的？某个属性
	- (10-9) 点的搜索 还是先ICP优化吧
- 第四个线程:
	- (10-9) //类写的差不多了
	- (10-12) //ICP 不好使 sprase ICP 还没读 好像还不错
	- (10-19) /sprase ICP 中间结果
		- (10-20) /添加点是否收敛的判断
		- (10-20) /bug 初始化三角化的点把Tracking中新建的点覆盖了 可能因为：ORBmatcher 极线搜索是不是太多重复了
		- 实现上的修改，两个Map之间直接做 注意多线程的问题
		- 匹配好的直接算位姿、搜索更多的匹配点
		- 未匹配好的SICP 搜索匹配点、这里应该需要一些判断来确定匹配关系
		- 后面加上scale微调的SICP
		- 所有目的是为了得到匹配点，最后用于联系reference与当前场景
	- (10-12) Eigen 输入的优化
	- super4pcs(scale 先由descritor确定)
	- sprase ICP 的尺度优化
- 如何处理Reference Image

- 机械臂
