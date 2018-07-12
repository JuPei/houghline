#include <opencv2\opencv.hpp>
#include"iostream"
using namespace cv;
using namespace std;
static void  //静态定义
HoughLineStandard(const Mat& img, float rho, float theta,
int threshold, std::vector<Vec2f>& lines, int linesMax,
double min_theta, double max_theta, Point start, Point end);//函数参数声明
static void
icvHoughLinesProbabilistic(CvMat* image, Mat& img,
float rho, float theta, int threshold,
int lineLength, int lineGap,
int linesMax, CvPoint start, CvPoint end, vector<vector<Vec4i>> result_lines);
struct  LinePolar
{
	float rho;
	float angle;
};
struct  line_2point
{
	Point start;
	Point end;
};

int main()
{
	Mat src = imread("2.jpg");
	Mat dd;
	src.copyTo(dd);
	Mat grayImage, thresholdImage;
	cvtColor(src, grayImage, COLOR_BGR2GRAY);
	threshold(grayImage, thresholdImage, 100, 255, 0);
	Point START,END;
	START.x = 700;
	START.y = 700;
	END.x = 1200;
	END.y = 1200;
	Rect R(START, END);
	rectangle(grayImage, R, Scalar(0, 0, 255), 2);
	imwrite("111.jpg", grayImage);
	Mat edge;
	Canny(thresholdImage, edge, 50, 200, 3);
	//定义输出数组，用于存储直线的角度和距离这两个变量
	vector<Vec2f> lines;

	//距离分辨率为1，角度分辨率为π/180，阈值为215
	//阈值的选取直接影响到输出直线的数量
	HoughLineStandard(edge, 1, CV_PI / 180, 650, lines, 20, 0, (CV_PI / 180) * 360, START, END);
	
    //HoughLines(edge, lines, 1, CV_PI / 180, 150, 0, 0);
	imwrite("edge.jpg", edge);
	//if (lines.size() == 0)
		//cout << "rho "<< endl;
	for (size_t i = 0; i < lines.size(); i++)
	{
		//提取出距离和角度
		float rho = lines[i][0], theta = lines[i][1];
		//cout << rho << endl;
		//定义两个点，两点确定一条直线
		//计算得到的两点的坐标为（ρcosθ-1000sinθ，ρsinθ+1000cosθ），（ρcosθ+1000sinθ，ρsinθ-1000cosθ）
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.y = 700;
		if (a == 0)
		{
			pt1.x = 700;
		}
		else
		{
			pt1.x = cvRound(rho - 700 * b);
		}
		pt2.y = 1200;
		pt2.x = cvRound(rho - 1200 * b);	
		//在原图上画宽带为2的红线
		line(src, pt1, pt2, Scalar(0, 0, 255), 1);
		imwrite("11.jpg", src);
	}
	vector<vector<Vec4i>> li;
	icvHoughLinesProbabilistic(&(CvMat)edge,dd, 1, CV_PI / 180, 200, 20, 5, 5, START, END,li);
	imwrite("dd.jpg", dd);
	waitKey(0);
	cin.get();
	return 1;
}
static void  //静态定义
HoughLineStandard(const Mat& img, float rho, float theta,
int threshold, std::vector<Vec2f>& lines, int linesMax,
double min_theta, double max_theta,Point start,Point end)//函数参数声明
{
	int i, j;//i为行，j为列
	float irho = 1 / rho;//单位ρ的倒数

	CV_Assert(img.type() == CV_8UC1);//检查图片是不是单通道8bit图像

	const uchar* image = img.ptr();//读取图片
	int step = (int)img.step;//图片每行像素所占字节
	int width = img.cols;//图片宽度
	int height = img.rows;//图片高度

	if (max_theta < min_theta) {
		CV_Error(CV_StsBadArg, "max_theta must be greater than min_theta");
	}//如果高阈值小于低阈值提示错误
	int numangle = cvRound((max_theta - min_theta) / theta);//参数空间θ个数
	int numrho = cvRound(((width + height) * 2 + 1) / rho);//参数空间ρ个数
	int*accum = new int[(numangle + 2) * (numrho + 2)]();
	std::vector<int> _sort_buf;
	float*sinTab = new float[numangle]();
	float*cosTab = new float[numangle]();

	float ang = static_cast<float>(min_theta);//ang为最小θ,类型为float
	for (int n = 0; n < numangle; ang += theta, n++)//对参数空间每个θ
	{
		sinTab[n] = (float)(sin((double)ang) * irho);//做好tabSin数组
		cosTab[n] = (float)(cos((double)ang) * irho);//做好tabCos数组
		//if (cosTab[n] == 0)
			//cout << "no";
	}
	for (i = start.y; i < end.y; i++)
		for (j = start.x; j < end.y; j++)//对于在图内的点
		{
		if (image[i * step + j] != 0)//如果二值图像非0点
			for (int n = 0; n < numangle; n++)

			{
			int r = cvRound(j * cosTab[n] + i * sinTab[n]);
			// 
			r += (numrho - 1) / 2;//距离偏移一半，ρ有负值，防止生成的索引产生覆盖
			accum[(n + 1) * (numrho + 2) + r + 1]++;//累加器相应单元+1
			//n+1是把第一行空出
			//r+1是把第一列空出
			//空出是防止邻域比较时溢出
			//numrho+2是总列数
			}
		}
	// stage 2. find local maximums
	for (int r = 0; r < numrho; r++)
		for (int n = 0; n < numangle; n++)
		{
		int base = (n + 1) * (numrho + 2) + r + 1;//累加器空间的索引

		if (accum[base] > threshold &&
			accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
			accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
			_sort_buf.push_back(base);//比较4邻域累加器中值的大小
		}
	std::sort(_sort_buf.begin(), _sort_buf.end());//降序排列
	//if (_sort_buf.size() == 0)
		//cout << "no";
	vector<int>::iterator it = _sort_buf.begin();
	// stage 4. store the first min(total,linesMax) lines to the output buffer
	linesMax = std::min(linesMax, (int)_sort_buf.size());
	double scale = 1. / (numrho + 2);
	//line_edge le;
	for (i = 0; i < linesMax; i++)
	{
		LinePolar line;
		int num = _sort_buf.size();
		int idx = _sort_buf[num-i-1];//排序后累加器空间accum的序列号
		int n = cvFloor(idx*scale) - 1;//获得行数
		int r = idx - (n + 1)*(numrho + 2) - 1;//获得列数
		line.rho = (r - (numrho - 1)*0.5f) * rho;//ρ
		line.angle = static_cast<float>(min_theta)+n * theta;//θ
		lines.push_back(Vec2f(line.rho, line.angle));//直线以(ρ,θ)装到lines中
	}
	delete[]sinTab;
	delete[]cosTab;
	delete[]accum;
}
static void
icvHoughLinesProbabilistic(CvMat* image,  Mat& img,
float rho, float theta, int threshold,
int lineLength, int lineGap,
int linesMax, CvPoint start, CvPoint end, vector<vector<Vec4i>> result_lines)
{
	//CvSeq *lines;
	//accum为累加器矩阵，mask为掩码矩阵
	cv::Mat accum, mask;
	cv::vector<float> sin_cos_tab;    //用于存储事先计算好的正弦和余弦值
	//开辟一段内存空间
	cv::MemStorage storage(cvCreateMemStorage(0));
	//用于存储特征点坐标，即边缘像素的位置
	CvSeq* seq;
	CvSeqWriter writer;
	int width, height;    //图像的宽和高
	int numangle, numrho;    //角度和距离的离散数量
	float ang;
	int r, n, count;
	CvPoint pt;
	float irho = 1 / rho;    //距离分辨率的倒数
	CvRNG rng = cvRNG(-1);    //随机数
	const float* psin_cos_tab;    //向量trigtab的地址指针
	uchar* pmask;    //矩阵mask的地址指针
	//确保输入图像的正确性
	CV_Assert(CV_IS_MAT(image) && CV_MAT_TYPE(image->type) == CV_8UC1);

	width = image->cols;    //提取出输入图像的宽
	height = image->rows;    //提取出输入图像的高
	//由角度和距离分辨率，得到角度和距离的离散数量
	numangle = cvRound(CV_PI / theta);
	numrho = cvRound(((width + height) * 2 + 1) / rho);
	//创建累加器矩阵，即霍夫空间
	accum.create(numangle, numrho, CV_32SC1);
	//创建掩码矩阵，大小与输入图像相同
	mask.create(height, width, CV_8UC1);
	//定义trigtab的大小，因为要存储正弦和余弦值，所以长度为角度离散数的2倍
	sin_cos_tab.resize(numangle * 2);
	//累加器矩阵清零
	accum = cv::Scalar(0);
	//避免重复计算，事先计算好所需的所有正弦和余弦值
	for (ang = 0, n = 0; n < numangle; ang += theta, n++)
	{
		sin_cos_tab[n * 2] = (float)(cos(ang) * irho);
		sin_cos_tab[n * 2 + 1] = (float)(sin(ang) * irho);
	}
	//赋值首地址
	psin_cos_tab = &sin_cos_tab[0];
	pmask = mask.data;
	//开始写入序列
	cvStartWriteSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage, &writer);

	// stage 1. collect non-zero image points
	//收集图像中的所有非零点，因为输入图像是边缘图像，所以非零点就是边缘点
	for (pt.y=start.y, count = 0; pt.y < end.y; pt.y++)
	{
		//提取出输入图像和掩码矩阵的每行地址指针
		const uchar* data = image->data.ptr + pt.y*image->step;
		uchar* mdata = pmask + pt.y*width;
		for (pt.x = start.x; pt.x < end.x; pt.x++)
		{
			if (data[pt.x])    //是边缘点
			{
				mdata[pt.x] = (uchar)1;    //掩码的相应位置置1
				CV_WRITE_SEQ_ELEM(pt, writer);    //把该坐标位置写入序列
			}
			else    //不是边缘点
				mdata[pt.x] = 0;    //掩码的相应位置清0
		}
	}
	//终止写序列，seq为所有边缘点坐标位置的序列
	seq = cvEndWriteSeq(&writer);
	count = seq->total;    //得到边缘点的数量

	// stage 2. process all the points in random order
	//随机处理所有的边缘点
	for (; count > 0; count--)
	{
		// choose random point out of the remaining ones
		//步骤1，在剩下的边缘点中随机选择一个点，idx为不大于count的随机数
		int idx = cvRandInt(&rng) % count;
		//max_val为累加器的最大值，max_n为最大值所对应的角度
		int max_val = threshold - 1, max_n = 0;
		//由随机数idx在序列中提取出所对应的坐标点
		CvPoint* point = (CvPoint*)cvGetSeqElem(seq, idx);
		//定义直线的两个端点
		CvPoint line_end[2] = { { 0, 0 }, { 0, 0 } };
		float a, b;
		//累加器的地址指针，也就是霍夫空间的地址指针
		int* adata = (int*)accum.data;
		int i, j, k, x0, y0, dx0, dy0, xflag;
		int good_line;
		const int shift = 16;
		//提取出坐标点的横、纵坐标
		i = point->y;
		j = point->x;

		// "remove" it by overriding it with the last element
		//用序列中的最后一个元素覆盖掉刚才提取出来的随机坐标点
		*point = *(CvPoint*)cvGetSeqElem(seq, count - 1);//将这次循环被选中的点在下一次随机中排除

		// check if it has been excluded already (i.e. belongs to some other line)
		//检测这个坐标点是否已经计算过，也就是它已经属于其他直线
		//因为计算过的坐标点会在掩码矩阵mask的相对应位置清零
		if (!pmask[i*width + j])    //该坐标点被处理过
			continue;    //不做任何处理，继续主循环

		// update accumulator, find the most probable line
		//步骤2，更新累加器矩阵，找到最有可能的直线
		for (n = 0; n < numangle; n++, adata += numrho)
		{
			//由角度计算距离
			r = cvRound(j * psin_cos_tab[n * 2] + i * psin_cos_tab[n * 2 + 1]);
			r += (numrho - 1) / 2;
			//在累加器矩阵的相应位置上数值加1，并赋值给val
			int val = ++adata[r];
			//更新最大值，并得到它的角度
			if (max_val < val)
			{
				max_val = val;
				max_n = n;
			}
		}

		// if it is too "weak" candidate, continue with another point
		//步骤3，如果上面得到的最大值小于阈值，则放弃该点，继续下一个点的计算
		if (max_val < threshold)
			continue;

		// from the current point walk in each direction
		// along the found line and extract the line segment
		//步骤4，从当前点出发，沿着它所在直线的方向前进，直到达到端点为止
		a = -psin_cos_tab[max_n * 2 + 1];    //a=-sinθ
		b = psin_cos_tab[max_n * 2];    //b=cosθ
		//当前点的横、纵坐标值
		x0 = j;
		y0 = i;
		//确定当前点所在直线的角度是在45度～135度之间，还是在0～45或135度～180度之间
		if (fabs(a) > fabs(b))    //在45度～135度之间
		{
			xflag = 1;    //置标识位，标识直线的粗略方向
			//确定横、纵坐标的位移量
			dx0 = a > 0 ? 1 : -1;
			dy0 = cvRound(b*(1 << shift) / fabs(a));
			//确定纵坐标
			y0 = (y0 << shift) + (1 << (shift - 1));
		}
		else    //在0～45或135度～180度之间
		{
			xflag = 0;   //清标识位
			//确定横、纵坐标的位移量
			dy0 = b > 0 ? 1 : -1;
			dx0 = cvRound(a*(1 << shift) / fabs(b));
			//确定横坐标
			x0 = (x0 << shift) + (1 << (shift - 1));
		}
		//搜索直线的两个端点
		for (k = 0; k < 2; k++)
		{
			//gap表示两条直线的间隙，x和y为搜索位置，dx和dy为位移量
			int gap = 0, x = x0, y = y0, dx = dx0, dy = dy0;
			//搜索第二个端点的时候，反方向位移
			if (k > 0)
				dx = -dx, dy = -dy;

			// walk along the line using fixed-point arithmetics,
			// stop at the image border or in case of too big gap
			//沿着直线的方向位移，直到到达图像的边界或大的间隙为止
			for (;; x += dx, y += dy)
			{
				uchar* mdata;
				int i1, j1;
				//确定新的位移后的坐标位置
				if (xflag)
				{
					j1 = x;
					i1 = y >> shift;
				}
				else
				{
					j1 = x >> shift;
					i1 = y;
				}
				//如果到达了图像的边界，停止位移，退出循环
				if (j1 < 0 || j1 >= width || i1 < 0 || i1 >= height)
					break;
				//定位位移后掩码矩阵位置
				mdata = pmask + i1*width + j1;

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				//该掩码不为0，说明该点可能是在直线上
				if (*mdata)
				{
					gap = 0;    //设置间隙为0
					//更新直线的端点位置
					line_end[k].y = i1;
					line_end[k].x = j1;
				}
				//掩码为0，说明不是直线，但仍继续位移，直到间隙大于所设置的阈值为止
				else if (++gap > lineGap)    //间隙加1
					break;
			}
		}
		//步骤5，由检测到的直线的两个端点粗略计算直线的长度
		//当直线长度大于所设置的阈值时，good_line为1，否则为0
		good_line = abs(line_end[1].x - line_end[0].x) >= lineLength ||
			abs(line_end[1].y - line_end[0].y) >= lineLength;
		//再次搜索端点，目的是更新累加器矩阵和更新掩码矩阵，以备下一次循环使用
		for (k = 0; k < 2; k++)
		{
			int x = x0, y = y0, dx = dx0, dy = dy0;

			if (k > 0)
				dx = -dx, dy = -dy;

			// walk along the line using fixed-point arithmetics,
			// stop at the image border or in case of too big gap
			for (;; x += dx, y += dy)
			{
				uchar* mdata;
				int i1, j1;

				if (xflag)
				{
					j1 = x;
					i1 = y >> shift;
				}
				else
				{
					j1 = x >> shift;
					i1 = y;
				}

				mdata = pmask + i1*width + j1;

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				if (*mdata)
				{
					//if语句的作用是清除那些已经判定是好的直线上的点对应的累加器的值，避免再次利用这些累加值
					if (good_line)    //在第一次搜索中已经确定是好的直线
					{
						//得到累加器矩阵地址指针
						adata = (int*)accum.data;
						for (n = 0; n < numangle; n++, adata += numrho)
						{
							r = cvRound(j1 * psin_cos_tab[n * 2] + i1 * psin_cos_tab[n * 2 + 1]);
							r += (numrho - 1) / 2;
							adata[r]--;    //相应的累加器减1
						}
					}
					//搜索过的位置，不管是好的直线，还是坏的直线，掩码相应位置都清0，这样下次就不会再重复搜索这些位置了，从而达到减小计算边缘点的目的
					*mdata = 0;
				}
				//如果已经到达了直线的端点，则退出循环
				if (i1 == line_end[k].y && j1 == line_end[k].x)
					break;
			}
		}
		//如果是好的直线
		if (good_line)
		{
			vector<Vec4i> lr = { line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
			//CvRect lr = { line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
			line(img, line_end[0], line_end[1], Scalar(0, 0, 255));
			result_lines.push_back(lr);
			//把两个端点压入序列中
			//cvSeqPush(lines, &lr);
			//如果检测到的直线数量大于阈值，则退出该函数
			//if (lines->total >= linesMax)
				//return;
		}
	}
}

