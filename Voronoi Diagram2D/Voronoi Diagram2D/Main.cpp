#include"Heder.h"

bool key1 = true;
bool key2 = false;
bool key3 = true;
bool key4 = false;

double width = WIDTH;
double height = HEIGHT;

Mesh* pDelaunayMesh;
Mesh* pVoronoiMesh;

int delaunayVertexId = 0;
int delaunayFaceId = 0;
int voronoiVertexId = 0;
int voronoiFaceId = 0;

//用来存储DelaunayMesh的所有点
vector<CVertex*> allVertex;

//用来存储ConvexHull算法产生的包围圈的点
vector<CVertex*> vecConvexHull;

//用来存储凸包内部的点
vector<CVertex*> toBeAdd;

//用来存储face对应的外接圆圆心
map<CFace*, MyPoint2> mpFaceCenter;



//随机数初始化Vertex
void initVertex();

//插入排序，从小到大，以x坐标大小排序，若x坐标相同，则按y坐标大小排序
void insertSort();

//向量积，大于0表示从oa到ob为逆时针旋转
GLfloat cross(CVertex* o, CVertex* a, CVertex* b);

//Andrew's Monotone Chain
void ConvexHull();

//计算圆心，输入为三个点
MyPoint2 getCenter(vector<CVertex*> vec);

//计算圆心，输入为一个面
MyPoint2 getTriCenter(CFace* pFace);

//将Face和相应的圆心Vertex加入mpFaceCenter中
void addMpFaceCenter(CFace* pFace);

//将Face和相应的圆心Vertex从mpFaceCenter中删除
void delMpFaceCenter(CFace* pFace);

//Euclidean Distance的平方
double eDistance(double x1, double y1, double x2, double y2);

//判断一个点是否在一个三角面的外接圆中
bool isCircleContain(CFace* pFace, CVertex* pVertex);

//根据ConvexHull生成的点集初始化Delaunay图
void initDelaunayMesh();

//判断是否为DelaunayTriangle
bool isDelaunayTriangle();

//Delaunay Triangulation
void DelaunayTriangulation();

//VoronoiDiagram算法
void VoronoiDiagram();

//帮助
void help();

//绘制点
void drawVertex(Mesh* pMesh);

//绘制边
void drawEdge(Mesh* pMesh);

//绘制面
void drawFace(Mesh* pMesh);

//OpenGL图像绘制
void renderScene();

//窗口改变
void changeSize(int w, int h);

//响应鼠标事件
void mouseClick(int button, int state, int x, int y);

//响应键盘事件
void keyBoardFunc(unsigned char key, int x, int y);

//初始化OpenGL
void init_openGL(int argc, char *argv[]);



//随机数初始化Vertex
void initVertex() {
	CVertex* tmp = NULL;
	while (delaunayVertexId < VERTEXNUM) {
		tmp = (*pDelaunayMesh).createVertex(delaunayVertexId++);

		tmp->point().coord()[0] = (rand() % (int)(2 * ACCURACY) - ACCURACY) / ACCURACY;
		tmp->point().coord()[1] = (rand() % (int)(2 * ACCURACY) - ACCURACY) / ACCURACY;
		allVertex.push_back(tmp);
	}
}

//插入排序，从小到大，以x坐标大小排序，若x坐标相同，则按y坐标大小排序
void insertSort() {
	int i, j;
	CVertex* tmp = NULL;
	for (i = 0; i < allVertex.size(); i++) {
		tmp = allVertex[i];
		j = i - 1;
		while (j >= 0 && allVertex[j]->point().coord()[0] >= tmp->point().coord()[0]) {
			if (allVertex[j]->point().coord()[0] == tmp->point().coord()[0]
				&& allVertex[j]->point().coord()[1] <= tmp->point().coord()[1]) {
				break;
			}
			allVertex[j + 1] = allVertex[j];
			j--;
		}
		allVertex[j + 1] = tmp;
	}
}

//向量积，大于0表示从oa到ob为逆时针旋转
GLfloat cross(CVertex* o, CVertex* a, CVertex* b) {
	return (a->point().coord()[0] - o->point().coord()[0])
		*(b->point().coord()[1] - o->point().coord()[1])
		- (a->point().coord()[1] - o->point().coord()[1])
		*(b->point().coord()[0] - o->point().coord()[0]);
}

//Andrew's Monotone Chain
void ConvexHull() {
	//将Vertex的顶点id排序
	insertSort();

	//记录包围点的个数
	int m = 0;
	int size = allVertex.size();

	//从左到右，得出下半部分包围点
	for (int i = 0; i < size; i++) {
		while (m >= 2 && cross(vecConvexHull[m - 2], vecConvexHull[m - 1], allVertex[i]) < 0) {
			vecConvexHull.pop_back();
			m--;
		}
		vecConvexHull.push_back(allVertex[i]);
		m++;
	}

	//从右到左，得出上半部分包围点，多存储了一次起点id
	for (int i = size - 2; i >= 0; i--) {
		while (cross(vecConvexHull[m - 2], vecConvexHull[m - 1], allVertex[i]) < 0) {
			vecConvexHull.pop_back();
			m--;
		}
		vecConvexHull.push_back(allVertex[i]);
		m++;
	}

	vecConvexHull.pop_back();
}

//计算圆心，输入为三个点
MyPoint2 getCenter(vector<CVertex*> vec) {
	//求出外接圆的圆心
	double x0 = vec[0]->point().coord()[0];
	double y0 = vec[0]->point().coord()[1];
	double x1 = vec[1]->point().coord()[0];
	double y1 = vec[1]->point().coord()[1];
	double x2 = vec[2]->point().coord()[0];
	double y2 = vec[2]->point().coord()[1];

	double a0 = (x0 + x1) / 2;
	double b0 = (y0 + y1) / 2;
	double a1 = (x2 + x1) / 2;
	double b1 = (y2 + y1) / 2;

	//横纵坐标
	double x;
	double y;


	if (y0 != y1 && y1 != y2) {
		double k1 = -(x0 - x1) / (y0 - y1);
		double c1 = b0 - k1 * a0;
		double k2 = -(x1 - x2) / (y1 - y2);
		double c2 = b1 - k2 * a1;

		x = (c2 - c1) / (k1 - k2);
		y = k1 * x + c1;
	}
	else if (y0 == y1) {
		x = a0;
		double k2 = -(x1 - x2) / (y1 - y2);
		double c2 = b1 - k2 * a1;
		y = k2 * x + c2;
	}
	else {
		x = a1;
		double k1 = -(x0 - x1) / (y0 - y1);
		double c1 = b0 - k1 * a0;
		y = k1 * x + c1;
	}

	MyPoint2 point2(x, y);

	return point2;
}

//计算圆心，输入为一个面
MyPoint2 getTriCenter(CFace* pFace) {
	vector<CVertex*> vec;
	for (fvIter iter(pFace); !iter.end(); iter++) {
		vec.push_back(*iter);
	}
	return getCenter(vec);
}

//将Face和相应的圆心Vertex加入mpFaceCenter中
void addMpFaceCenter(CFace* pFace) {
	MyPoint2 point2 = getTriCenter(pFace);
	mpFaceCenter.insert(pair<CFace*, MyPoint2>(pFace, point2));
}

//将Face和相应的圆心Vertex从mpFaceCenter中删除
void delMpFaceCenter(CFace* pFace) {
	mpFaceCenter.erase(pFace);
}

//Euclidean Distance的平方
double eDistance(double x1, double y1, double x2, double y2) {
	return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}

//判断一个点是否在一个三角面的外接圆中
bool isCircleContain(CFace* pFace, CVertex* pVertex) {
	MyPoint2 point2 = mpFaceCenter.at(pFace);
	fvIter iter(pFace);

	double r = eDistance(point2.x, point2.y, (*iter)->point().coord()[0], (*iter)->point().coord()[1]);
	double distance = eDistance(point2.x, point2.y, pVertex->point().coord()[0], pVertex->point().coord()[1]);

	return r > distance;
}

//根据ConvexHull生成的点集初始化Delaunay图
void initDelaunayMesh() {
	vector<CVertex*> vecTmp = vecConvexHull;
	bool isContainOther = false;
	while (vecTmp.size() > 3) {
		int size = vecTmp.size();
		for (int i = 0; i < size; i++) {
			isContainOther = false;
			vector<CVertex*> fv;
			fv.push_back(vecTmp[i]);
			fv.push_back(vecTmp[(i + 1) % size]);
			fv.push_back(vecTmp[(i + 2) % size]);

			MyPoint2 center = getCenter(fv);
			double r = eDistance(center.x, center.y, fv[0]->point().coord()[0], fv[0]->point().coord()[1]);

			for (int j = 0; j < vecConvexHull.size(); j++) {
				CVertex* pVertexTmp = vecConvexHull[j];
				if (pVertexTmp != fv[0] && pVertexTmp != fv[1] && pVertexTmp != fv[2]
					&& r > eDistance(center.x, center.y, pVertexTmp->point().coord()[0], pVertexTmp->point().coord()[1])) {
					isContainOther = true;
					break;
				}
			}
			if (!isContainOther) {
				mpFaceCenter.insert(pair<CFace*, MyPoint2>((*pDelaunayMesh).createFace(fv, delaunayFaceId++), center));
				vecTmp.erase(vecTmp.begin() + (i + 1) % size);
				break;
			}
		}
	}

	addMpFaceCenter((*pDelaunayMesh).createFace(vecTmp, delaunayFaceId++));
}

//判断是否为DelaunayTriangle
bool isDelaunayTriangle() {
	for (mfIter iter(pDelaunayMesh); !iter.end(); iter++) {
		fvIter iter2(*iter);
		CVertex* v1 = *iter2;
		iter2++;
		CVertex* v2 = *iter2;
		iter2++;
		CVertex* v3 = *iter2;
		for (int i = 0; i < allVertex.size(); i++) {
			if (v1 != allVertex[i] && v2 != allVertex[i] && v3 != allVertex[i] && isCircleContain(*iter, allVertex[i])) {
				return false;
			}
		}
	}
	return true;
}

//Delaunay Triangulation
void DelaunayTriangulation() {
	//初始化toBeAdd
	for (int i = 0; i < allVertex.size(); i++) {
		int num = count(vecConvexHull.begin(), vecConvexHull.end(), allVertex[i]);
		if (num == 0) {
			toBeAdd.push_back(allVertex[i]);
		}
	}

	//用包围点和里面的一点创建三角面
	initDelaunayMesh();

	//逐个向DelaunayMesh中添加点
	for (int n = 0; n < toBeAdd.size(); n++) {

		//取出所有外接圆包含当前点的Face
		vector<CFace*> vecFace;
		for (mfIter iter(pDelaunayMesh); !iter.end(); iter++) {
			if (isCircleContain(*iter, toBeAdd[n])) {
				vecFace.push_back(*iter);
			}
		}

		//取出上一步得到的所有Face相互之间不共享的边的端点,并按逆时针顺序加入vecVerTmp中
		vector<CVertex*> vecVerTmp;
		for (int i = 0; i < vecFace.size(); i++) {
			CFace* f0 = vecFace[i];
			for (feIter iter(f0); !iter.end(); iter++) {
				CEdge* tmpEdge = *iter;
				CHalfEdge* h0 = tmpEdge->halfedge(0);
				CHalfEdge* h1 = tmpEdge->halfedge(1);
				assert(h0 != NULL);
				if (h1 == NULL) {
					vecVerTmp.push_back(h0->source());
					vecVerTmp.push_back(h0->target());
				}
				else {
					CFace* f1 = (f0 == (*pDelaunayMesh).edgeFace1(tmpEdge)) 
						? (*pDelaunayMesh).edgeFace2(tmpEdge) 
						: (*pDelaunayMesh).edgeFace1(tmpEdge);
					vector<CFace*>::iterator tmpIter = find(vecFace.begin(), vecFace.end(), f1);
					if (tmpIter == vecFace.end()) {

						CHalfEdge* tmpHe = f0->halfedge();
						while (tmpHe != h0 && tmpHe != h1) {
							tmpHe = tmpHe->he_next();
						}
						vecVerTmp.push_back(tmpHe->source());
						vecVerTmp.push_back(tmpHe->target());
					}
				}
			}
		}

		//删除所有第一步得到的Face
		for (int i = 0; i < vecFace.size(); i++) {
			delMpFaceCenter(vecFace[i]);
			(*pDelaunayMesh).deleteFace(vecFace[i]);
		}

		//每个不共享边的两个端点与当前点形成新的Face
		for (int i = 0; i < vecVerTmp.size(); i += 2) {
			vector<CVertex*> tmp;
			tmp.push_back(toBeAdd[n]);
			tmp.push_back(vecVerTmp[i]);
			tmp.push_back(vecVerTmp[i + 1]);
			addMpFaceCenter((*pDelaunayMesh).createFace(tmp, delaunayFaceId++));
		}
	}
}

//VoronoiDiagram算法
void VoronoiDiagram() {
	ConvexHull();
	DelaunayTriangulation();

	(*pDelaunayMesh).labelBoundary();

	//将DelaunayMesh中的Face与VoronoiMesh中的点对应起来
	map<CFace*, CVertex*> mpFaceVertex;
	for (mfIter iter(pDelaunayMesh); !iter.end(); iter++) {
		CVertex* tmp = (*pVoronoiMesh).createVertex(voronoiVertexId++);
		tmp->point().coord()[0] = mpFaceCenter.at(*iter).x;
		tmp->point().coord()[1] = mpFaceCenter.at(*iter).y;

		mpFaceVertex.insert(pair<CFace*, CVertex*>(*iter, tmp));
	}

	//遍历DelaunayMesh中包含toBeAdd中一点的所有Face，得到VoronoiMesh中的点对应的并生成Face
	for (int i = 0; i < toBeAdd.size(); i++) {
		vector<CVertex*> vecTmp;

		for (vfIter iter(toBeAdd[i]); !iter.end(); iter++) {
			vecTmp.push_back(mpFaceVertex.at(*iter));
		}

		(*pVoronoiMesh).createFace(vecTmp, voronoiFaceId++);
	}

	//得到位于Mesh边缘的Edge，生成垂直平分线上的另一点
	map<CEdge*, CVertex*> mpEdgeVertex;
	for (int i = 0; i < vecConvexHull.size(); i++) {
		for (vfIter iter(vecConvexHull[i]); !iter.end(); iter++) {
			CFace* faceTmp = *iter;

			for (feIter iter(faceTmp); !iter.end(); iter++) {
				CEdge* edgeTmp = *iter;
				if (edgeTmp->boundary()) {

					CVertex* center = mpFaceVertex.at(faceTmp);

					CVertex* v0 = (*pDelaunayMesh).edgeVertex1(edgeTmp);
					CVertex* v1 = (*pDelaunayMesh).edgeVertex2(edgeTmp);

					CVertex* newVert = (*pVoronoiMesh).createVertex(voronoiVertexId++);
					mpEdgeVertex.insert(pair<CEdge*, CVertex*>(edgeTmp, newVert));

					double x = center->point().coord()[0];
					double y = center->point().coord()[1];

					double x0 = v0->point().coord()[0];
					double y0 = v0->point().coord()[1];
					double x1 = v1->point().coord()[0];
					double y1 = v1->point().coord()[1];

					if (y0 == y1) {
						newVert->point().coord()[0] = x;
						if (y > 0) {
							newVert->point().coord()[1] = BORDER;
						}
						else {
							newVert->point().coord()[1] = -BORDER;
						}
					}
					else if (x0 == x1) {
						newVert->point().coord()[1] = y;
						if (x > 0) {
							newVert->point().coord()[0] = BORDER;
						}
						else {
							newVert->point().coord()[0] = -BORDER;
						}
					}
					else {
						double k = -(x0 - x1) / (y0 - y1);
						double b = y - k * x;

						double xm = (x0 + x1) / 2.0;
						double ym = (y0 + y1) / 2.0;

						if (xm > 0 && ym>0) {
							if (k<1 && k>-1) {
								newVert->point().coord()[0] = BORDER;
								newVert->point().coord()[1] = BORDER * k + b;
							}
							else {
								newVert->point().coord()[1] = BORDER;
								newVert->point().coord()[0] = (BORDER - b) / k;
							}
						}
						else if (xm > 0 && ym < 0) {
							if (k<1 && k>-1) {
								newVert->point().coord()[0] = BORDER;
								newVert->point().coord()[1] = BORDER * k + b;
							}
							else {
								newVert->point().coord()[1] = -BORDER;
								newVert->point().coord()[0] = (-BORDER - b) / k;
							}
						}
						else if (xm < 0 && ym>0) {
							if (k<1 && k>-1) {
								newVert->point().coord()[0] = -BORDER;
								newVert->point().coord()[1] = -BORDER * k + b;
							}
							else {
								newVert->point().coord()[1] = BORDER;
								newVert->point().coord()[0] = (BORDER - b) / k;
							}
						}
						else {
							if (k<1 && k>-1) {
								newVert->point().coord()[0] = -BORDER;
								newVert->point().coord()[1] = -BORDER * k + b;
							}
							else {
								newVert->point().coord()[1] = -BORDER;
								newVert->point().coord()[0] = (-BORDER - b) / k;
							}
						}
					}
				}
			}
		}
	}

	//遍历DelaunayMesh中包含vecConvexHull中一点的所有Face
	for (int i = 0; i < vecConvexHull.size(); i++) {
		vector<CVertex*> vecTmp;

		vfIter iter(vecConvexHull[i]);
		CFace* startFace = *iter;
		CVertex* startVertex = mpFaceVertex.at(startFace);
		for (feIter startIter(startFace); !startIter.end(); startIter++) {
			CEdge* edgeTmp = *startIter;
			if (edgeTmp->boundary()) {
				vecTmp.push_back(mpEdgeVertex.at(edgeTmp));
			}
		}
		vecTmp.push_back(startVertex);
		iter++;

		for (; !iter.end(); iter++) {
			CFace* faceTmp = *iter;
			CVertex* center = mpFaceVertex.at(faceTmp);
			assert(center != NULL);
			vecTmp.push_back(center);
			for (feIter iter2(faceTmp); !iter2.end(); iter2++) {
				CEdge* edgeTmp = *iter2;
				if (edgeTmp->boundary()) {
					vecTmp.push_back(mpEdgeVertex.at(edgeTmp));
				}
			}
		}

		(*pVoronoiMesh).createFace(vecTmp, voronoiFaceId++);
	}

	//(*pVoronoiMesh).labelBoundary();
}

int main(int argc, char*argv[])
{
	srand((unsigned)time(NULL));
	pDelaunayMesh = new Mesh;
	pVoronoiMesh = new Mesh;
	initVertex();
	VoronoiDiagram();
	init_openGL(argc, argv);
	return 0;
}