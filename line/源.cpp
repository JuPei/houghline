#include <opencv2\opencv.hpp>
#include"iostream"
using namespace cv;
using namespace std;
static void  //��̬����
HoughLineStandard(const Mat& img, float rho, float theta,
int threshold, std::vector<Vec2f>& lines, int linesMax,
double min_theta, double max_theta, Point start, Point end);//������������
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
	//����������飬���ڴ洢ֱ�ߵĽǶȺ;�������������
	vector<Vec2f> lines;

	//����ֱ���Ϊ1���Ƕȷֱ���Ϊ��/180����ֵΪ215
	//��ֵ��ѡȡֱ��Ӱ�쵽���ֱ�ߵ�����
	HoughLineStandard(edge, 1, CV_PI / 180, 650, lines, 20, 0, (CV_PI / 180) * 360, START, END);
	
    //HoughLines(edge, lines, 1, CV_PI / 180, 150, 0, 0);
	imwrite("edge.jpg", edge);
	//if (lines.size() == 0)
		//cout << "rho "<< endl;
	for (size_t i = 0; i < lines.size(); i++)
	{
		//��ȡ������ͽǶ�
		float rho = lines[i][0], theta = lines[i][1];
		//cout << rho << endl;
		//���������㣬����ȷ��һ��ֱ��
		//����õ������������Ϊ����cos��-1000sin�ȣ���sin��+1000cos�ȣ�������cos��+1000sin�ȣ���sin��-1000cos�ȣ�
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
		//��ԭͼ�ϻ����Ϊ2�ĺ���
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
static void  //��̬����
HoughLineStandard(const Mat& img, float rho, float theta,
int threshold, std::vector<Vec2f>& lines, int linesMax,
double min_theta, double max_theta,Point start,Point end)//������������
{
	int i, j;//iΪ�У�jΪ��
	float irho = 1 / rho;//��λ�ѵĵ���

	CV_Assert(img.type() == CV_8UC1);//���ͼƬ�ǲ��ǵ�ͨ��8bitͼ��

	const uchar* image = img.ptr();//��ȡͼƬ
	int step = (int)img.step;//ͼƬÿ��������ռ�ֽ�
	int width = img.cols;//ͼƬ���
	int height = img.rows;//ͼƬ�߶�

	if (max_theta < min_theta) {
		CV_Error(CV_StsBadArg, "max_theta must be greater than min_theta");
	}//�������ֵС�ڵ���ֵ��ʾ����
	int numangle = cvRound((max_theta - min_theta) / theta);//�����ռ�ȸ���
	int numrho = cvRound(((width + height) * 2 + 1) / rho);//�����ռ�Ѹ���
	int*accum = new int[(numangle + 2) * (numrho + 2)]();
	std::vector<int> _sort_buf;
	float*sinTab = new float[numangle]();
	float*cosTab = new float[numangle]();

	float ang = static_cast<float>(min_theta);//angΪ��С��,����Ϊfloat
	for (int n = 0; n < numangle; ang += theta, n++)//�Բ����ռ�ÿ����
	{
		sinTab[n] = (float)(sin((double)ang) * irho);//����tabSin����
		cosTab[n] = (float)(cos((double)ang) * irho);//����tabCos����
		//if (cosTab[n] == 0)
			//cout << "no";
	}
	for (i = start.y; i < end.y; i++)
		for (j = start.x; j < end.y; j++)//������ͼ�ڵĵ�
		{
		if (image[i * step + j] != 0)//�����ֵͼ���0��
			for (int n = 0; n < numangle; n++)

			{
			int r = cvRound(j * cosTab[n] + i * sinTab[n]);
			// 
			r += (numrho - 1) / 2;//����ƫ��һ�룬���и�ֵ����ֹ���ɵ�������������
			accum[(n + 1) * (numrho + 2) + r + 1]++;//�ۼ�����Ӧ��Ԫ+1
			//n+1�ǰѵ�һ�пճ�
			//r+1�ǰѵ�һ�пճ�
			//�ճ��Ƿ�ֹ����Ƚ�ʱ���
			//numrho+2��������
			}
		}
	// stage 2. find local maximums
	for (int r = 0; r < numrho; r++)
		for (int n = 0; n < numangle; n++)
		{
		int base = (n + 1) * (numrho + 2) + r + 1;//�ۼ����ռ������

		if (accum[base] > threshold &&
			accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
			accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
			_sort_buf.push_back(base);//�Ƚ�4�����ۼ�����ֵ�Ĵ�С
		}
	std::sort(_sort_buf.begin(), _sort_buf.end());//��������
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
		int idx = _sort_buf[num-i-1];//������ۼ����ռ�accum�����к�
		int n = cvFloor(idx*scale) - 1;//�������
		int r = idx - (n + 1)*(numrho + 2) - 1;//�������
		line.rho = (r - (numrho - 1)*0.5f) * rho;//��
		line.angle = static_cast<float>(min_theta)+n * theta;//��
		lines.push_back(Vec2f(line.rho, line.angle));//ֱ����(��,��)װ��lines��
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
	//accumΪ�ۼ�������maskΪ�������
	cv::Mat accum, mask;
	cv::vector<float> sin_cos_tab;    //���ڴ洢���ȼ���õ����Һ�����ֵ
	//����һ���ڴ�ռ�
	cv::MemStorage storage(cvCreateMemStorage(0));
	//���ڴ洢���������꣬����Ե���ص�λ��
	CvSeq* seq;
	CvSeqWriter writer;
	int width, height;    //ͼ��Ŀ�͸�
	int numangle, numrho;    //�ǶȺ;������ɢ����
	float ang;
	int r, n, count;
	CvPoint pt;
	float irho = 1 / rho;    //����ֱ��ʵĵ���
	CvRNG rng = cvRNG(-1);    //�����
	const float* psin_cos_tab;    //����trigtab�ĵ�ַָ��
	uchar* pmask;    //����mask�ĵ�ַָ��
	//ȷ������ͼ�����ȷ��
	CV_Assert(CV_IS_MAT(image) && CV_MAT_TYPE(image->type) == CV_8UC1);

	width = image->cols;    //��ȡ������ͼ��Ŀ�
	height = image->rows;    //��ȡ������ͼ��ĸ�
	//�ɽǶȺ;���ֱ��ʣ��õ��ǶȺ;������ɢ����
	numangle = cvRound(CV_PI / theta);
	numrho = cvRound(((width + height) * 2 + 1) / rho);
	//�����ۼ������󣬼�����ռ�
	accum.create(numangle, numrho, CV_32SC1);
	//����������󣬴�С������ͼ����ͬ
	mask.create(height, width, CV_8UC1);
	//����trigtab�Ĵ�С����ΪҪ�洢���Һ�����ֵ�����Գ���Ϊ�Ƕ���ɢ����2��
	sin_cos_tab.resize(numangle * 2);
	//�ۼ�����������
	accum = cv::Scalar(0);
	//�����ظ����㣬���ȼ����������������Һ�����ֵ
	for (ang = 0, n = 0; n < numangle; ang += theta, n++)
	{
		sin_cos_tab[n * 2] = (float)(cos(ang) * irho);
		sin_cos_tab[n * 2 + 1] = (float)(sin(ang) * irho);
	}
	//��ֵ�׵�ַ
	psin_cos_tab = &sin_cos_tab[0];
	pmask = mask.data;
	//��ʼд������
	cvStartWriteSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage, &writer);

	// stage 1. collect non-zero image points
	//�ռ�ͼ���е����з���㣬��Ϊ����ͼ���Ǳ�Եͼ�����Է������Ǳ�Ե��
	for (pt.y=start.y, count = 0; pt.y < end.y; pt.y++)
	{
		//��ȡ������ͼ�����������ÿ�е�ַָ��
		const uchar* data = image->data.ptr + pt.y*image->step;
		uchar* mdata = pmask + pt.y*width;
		for (pt.x = start.x; pt.x < end.x; pt.x++)
		{
			if (data[pt.x])    //�Ǳ�Ե��
			{
				mdata[pt.x] = (uchar)1;    //�������Ӧλ����1
				CV_WRITE_SEQ_ELEM(pt, writer);    //�Ѹ�����λ��д������
			}
			else    //���Ǳ�Ե��
				mdata[pt.x] = 0;    //�������Ӧλ����0
		}
	}
	//��ֹд���У�seqΪ���б�Ե������λ�õ�����
	seq = cvEndWriteSeq(&writer);
	count = seq->total;    //�õ���Ե�������

	// stage 2. process all the points in random order
	//����������еı�Ե��
	for (; count > 0; count--)
	{
		// choose random point out of the remaining ones
		//����1����ʣ�µı�Ե�������ѡ��һ���㣬idxΪ������count�������
		int idx = cvRandInt(&rng) % count;
		//max_valΪ�ۼ��������ֵ��max_nΪ���ֵ����Ӧ�ĽǶ�
		int max_val = threshold - 1, max_n = 0;
		//�������idx����������ȡ������Ӧ�������
		CvPoint* point = (CvPoint*)cvGetSeqElem(seq, idx);
		//����ֱ�ߵ������˵�
		CvPoint line_end[2] = { { 0, 0 }, { 0, 0 } };
		float a, b;
		//�ۼ����ĵ�ַָ�룬Ҳ���ǻ���ռ�ĵ�ַָ��
		int* adata = (int*)accum.data;
		int i, j, k, x0, y0, dx0, dy0, xflag;
		int good_line;
		const int shift = 16;
		//��ȡ�������ĺᡢ������
		i = point->y;
		j = point->x;

		// "remove" it by overriding it with the last element
		//�������е����һ��Ԫ�ظ��ǵ��ղ���ȡ��������������
		*point = *(CvPoint*)cvGetSeqElem(seq, count - 1);//�����ѭ����ѡ�еĵ�����һ��������ų�

		// check if it has been excluded already (i.e. belongs to some other line)
		//������������Ƿ��Ѿ��������Ҳ�������Ѿ���������ֱ��
		//��Ϊ����������������������mask�����Ӧλ������
		if (!pmask[i*width + j])    //������㱻�����
			continue;    //�����κδ���������ѭ��

		// update accumulator, find the most probable line
		//����2�������ۼ��������ҵ����п��ܵ�ֱ��
		for (n = 0; n < numangle; n++, adata += numrho)
		{
			//�ɽǶȼ������
			r = cvRound(j * psin_cos_tab[n * 2] + i * psin_cos_tab[n * 2 + 1]);
			r += (numrho - 1) / 2;
			//���ۼ����������Ӧλ������ֵ��1������ֵ��val
			int val = ++adata[r];
			//�������ֵ�����õ����ĽǶ�
			if (max_val < val)
			{
				max_val = val;
				max_n = n;
			}
		}

		// if it is too "weak" candidate, continue with another point
		//����3���������õ������ֵС����ֵ��������õ㣬������һ����ļ���
		if (max_val < threshold)
			continue;

		// from the current point walk in each direction
		// along the found line and extract the line segment
		//����4���ӵ�ǰ�����������������ֱ�ߵķ���ǰ����ֱ���ﵽ�˵�Ϊֹ
		a = -psin_cos_tab[max_n * 2 + 1];    //a=-sin��
		b = psin_cos_tab[max_n * 2];    //b=cos��
		//��ǰ��ĺᡢ������ֵ
		x0 = j;
		y0 = i;
		//ȷ����ǰ������ֱ�ߵĽǶ�����45�ȡ�135��֮�䣬������0��45��135�ȡ�180��֮��
		if (fabs(a) > fabs(b))    //��45�ȡ�135��֮��
		{
			xflag = 1;    //�ñ�ʶλ����ʶֱ�ߵĴ��Է���
			//ȷ���ᡢ�������λ����
			dx0 = a > 0 ? 1 : -1;
			dy0 = cvRound(b*(1 << shift) / fabs(a));
			//ȷ��������
			y0 = (y0 << shift) + (1 << (shift - 1));
		}
		else    //��0��45��135�ȡ�180��֮��
		{
			xflag = 0;   //���ʶλ
			//ȷ���ᡢ�������λ����
			dy0 = b > 0 ? 1 : -1;
			dx0 = cvRound(a*(1 << shift) / fabs(b));
			//ȷ��������
			x0 = (x0 << shift) + (1 << (shift - 1));
		}
		//����ֱ�ߵ������˵�
		for (k = 0; k < 2; k++)
		{
			//gap��ʾ����ֱ�ߵļ�϶��x��yΪ����λ�ã�dx��dyΪλ����
			int gap = 0, x = x0, y = y0, dx = dx0, dy = dy0;
			//�����ڶ����˵��ʱ�򣬷�����λ��
			if (k > 0)
				dx = -dx, dy = -dy;

			// walk along the line using fixed-point arithmetics,
			// stop at the image border or in case of too big gap
			//����ֱ�ߵķ���λ�ƣ�ֱ������ͼ��ı߽���ļ�϶Ϊֹ
			for (;; x += dx, y += dy)
			{
				uchar* mdata;
				int i1, j1;
				//ȷ���µ�λ�ƺ������λ��
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
				//���������ͼ��ı߽磬ֹͣλ�ƣ��˳�ѭ��
				if (j1 < 0 || j1 >= width || i1 < 0 || i1 >= height)
					break;
				//��λλ�ƺ��������λ��
				mdata = pmask + i1*width + j1;

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				//�����벻Ϊ0��˵���õ��������ֱ����
				if (*mdata)
				{
					gap = 0;    //���ü�϶Ϊ0
					//����ֱ�ߵĶ˵�λ��
					line_end[k].y = i1;
					line_end[k].x = j1;
				}
				//����Ϊ0��˵������ֱ�ߣ����Լ���λ�ƣ�ֱ����϶���������õ���ֵΪֹ
				else if (++gap > lineGap)    //��϶��1
					break;
			}
		}
		//����5���ɼ�⵽��ֱ�ߵ������˵���Լ���ֱ�ߵĳ���
		//��ֱ�߳��ȴ��������õ���ֵʱ��good_lineΪ1������Ϊ0
		good_line = abs(line_end[1].x - line_end[0].x) >= lineLength ||
			abs(line_end[1].y - line_end[0].y) >= lineLength;
		//�ٴ������˵㣬Ŀ���Ǹ����ۼ�������͸�����������Ա���һ��ѭ��ʹ��
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
					//if���������������Щ�Ѿ��ж��Ǻõ�ֱ���ϵĵ��Ӧ���ۼ�����ֵ�������ٴ�������Щ�ۼ�ֵ
					if (good_line)    //�ڵ�һ���������Ѿ�ȷ���Ǻõ�ֱ��
					{
						//�õ��ۼ��������ַָ��
						adata = (int*)accum.data;
						for (n = 0; n < numangle; n++, adata += numrho)
						{
							r = cvRound(j1 * psin_cos_tab[n * 2] + i1 * psin_cos_tab[n * 2 + 1]);
							r += (numrho - 1) / 2;
							adata[r]--;    //��Ӧ���ۼ�����1
						}
					}
					//��������λ�ã������Ǻõ�ֱ�ߣ����ǻ���ֱ�ߣ�������Ӧλ�ö���0�������´ξͲ������ظ�������Щλ���ˣ��Ӷ��ﵽ��С�����Ե���Ŀ��
					*mdata = 0;
				}
				//����Ѿ�������ֱ�ߵĶ˵㣬���˳�ѭ��
				if (i1 == line_end[k].y && j1 == line_end[k].x)
					break;
			}
		}
		//����Ǻõ�ֱ��
		if (good_line)
		{
			vector<Vec4i> lr = { line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
			//CvRect lr = { line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
			line(img, line_end[0], line_end[1], Scalar(0, 0, 255));
			result_lines.push_back(lr);
			//�������˵�ѹ��������
			//cvSeqPush(lines, &lr);
			//�����⵽��ֱ������������ֵ�����˳��ú���
			//if (lines->total >= linesMax)
				//return;
		}
	}
}

