//*****输入初始坐标*****
var x1=prompt("输入起始点的横坐标:");
var y1=prompt("输入起始点的纵坐标:");
var x2=prompt("输入终止点的横坐标:");
var y2=prompt("输入终止点的纵坐标:");

//*****参数*****
var K=100;  //迭代次数
var M=50;  //蚂蚁个数
var Alpha=1;  //Alpha表征信息素重要程度的参数
var Beta=7;  //Beta表征启发式因子重要程度的参数
var Rho=0.3;  //Rho信息素蒸发系数
var Q=1;  //信息素增加强度系数
var MM=23;   //地图长宽
var snode=new Node(x1,y1,);  //最短路径的起始点
var enode=new Node(x2,y2,);  //最短路径的目标点
var mink=0;  //最短路径的迭代次数
var minl=0;  //最短路径的蚂蚁代号
var minkl=0;  //最短路径的长度

//*****初始信息素矩阵*****
function initPheromoneMatrix(MM) 
{
    var pheromoneMatrix=new Array(MM);
    for (var i=0; i<MM; i++) 
    {
        pheromoneMatrix[i]=new Array(MM);
        for (var j=0; j<MM; j++) 
        {
            pheromoneMatrix[i][j]=1;
        }
    }
    return(pheromoneMatrix);
}

//*****以下是启发式信息矩阵*****    启发式信息:取为至目标点的直线距离的倒数
function initEta()
{
    var Eta=new Array(MM*MM);
    for(var i=0;i<MM;i++)
    {
        for(var j=0;j<MM;j++)
        {Eta(i*MM+1+j)=1/((ix-Ex)^2+(iy-Ey)^2)^0.5;}
    }
    return(Eta);
}

//*****储存每一只蚂蚁的爬行路线 爬行路径长度*****
for(var k=0;k<K;k++)
{
    var path=new Array(K);
    path[k]=new Array(M);
    var pathlength=new Array(K);  
    pathlength[k]=new Array(M);
    for(var m=1;m<M;m++)
    {
        path[k][m]=new Array();
    }
}

//信息素矩阵初始化
var PheromoneMatrix=initPheromoneMatrix(MM);

//计算一条路径的长度(若可走方向为8个，则要改动)
function one_path_length(path)
{return(len(path)*1.4)}

//查询能否走通
function checkPath(x,y,parent,enode,cost){
    var map=this.astar.map;
    var closeList=this.astar.closeList;
    var openList=this.astar.openList;
    var node=new Node(x,y,parent);
    if(map[x][y]==0) 
    {
        //地图元素是0则不能通过
        closeList.push(node);
        return false;
    }
    if(this.isListContains(closeList, x, y)!=-1)
    {
        //关闭列表中存在也不能通过
        return false;
    }
    var index=-1;
    if((index=this.isListContains(openList,x,y))!=-1) 
    {
        //开启列表中存在
        if((parent.getG()+cost)<openList[index].getG()) 
        {
            //当前路点g更小，把列表中的路点更新为当前点
            node.setParent(parent);
            this.countG(node, enode, cost);
            this.countF(node);
            openList[index]=node;
        }
    } 
    else 
    {
        node.setParent(parent);
        this.count(node,enode,cost);
        openList.push(node);//添加到开启列表
    }
    return true;
}

//寻找下一步可走的点
function available_node(node)
{
    var openList;
    if((node.getX()-1)>=0) 
    {checkPath(node.getX()-1,node.getY(),node, enode, COST_STRAIGHT);}//向左移动
    if((node.getX()+1)<row) 
    {checkPath(node.getX()+1,node.getY(),node, enode, COST_STRAIGHT);}//向右移动
    if((node.getY()-1)>=0) 
    {checkPath(node.getX(),node.getY()-1,node, enode, COST_STRAIGHT);}//向上移动
    if((node.getY()+1)<col) 
    {checkPath(node.getX(),node.getY()+1,node, enode, COST_STRAIGHT);}//向下移动
    return(openlist);
}

//赌轮转盘 选openlist中最好的node
function pick_node(openlist)
{
    var i=len(openlist);
    var PP=[];
    for(var j=0;j<i;j++)
    {
        var x=openlist[j].x;
        var y=openlist[j].y;
        PP.push((PheromoneMatrix[x,y])^Alpha*(Eta(y*MM+x+1))^Beta);
    }
    var sumpp=sum(PP);
    for(var j=0;j<i;j++)
    {
        PP[j]=PP[j]/sumpp;
    }
    var q=random(0,1);
    var a=0;
    for(a;a<i;a++)
    {
        if(PP[a]>=q)
        {break;}
    }
    return(openlist[a]);
}

//从终点回朔到起点
function getPath(resultList,node) 
{
    if(node.getParent()!=null) 
    {
        this.getPath(resultList,node.getParent());
    }
    resultList.push(node);
}

//寻找最短路径
function search(snode,enode)
{
    //初始化
    var Eta=initEta();
    //派出K轮蚂蚁
    for(var k=1;k<K;k++)
    {
        //信息素更新量初始化
        var Delta_PheromoneMatrix=new Array(MM);
        for(var i=0;i<MM;i++)
        {
            Delta_PheromoneMatrix[i]=new Array(MM);
            for(var j=0;j<MM;j++)
            {Delta_PheromoneMatrix[i][j]=0;}
        }
        //每轮派出M只蚂蚁
        for(var m=0;m<M;m++)
        {
            //*****状态初始化*****
            var nnode=new Node(snode.x,snode.y,);//当前节点为起始点
            path[k][m].push(snode);//爬行路径初始化
            pathlength[k][m]=1;//爬行路径长度初始化
            //禁忌表初始化
            var closelist=new Array();
            //已经在起始点，因此要排除
            closelist.push(snode);
            //邻接矩阵初始化
        

            //*****找路径*****
            var p=0;
            var anode=new Node(snode.x,snode.y,);//下一最优节点
            while((nnode.x!=enode.x)||(nnode.y!=enode.y))
            {
                var openlist=[]; //openlist中储存下一步能走的地方
                openlist=available_node(node);
                if(len(openlist)==0)
                {break;}//下一步没有地方可走
                anode=pick_node(openlist);
                //***状态更新***
                path[k][m].push(anode);//路径增加
                anode.parent=nnode;//记录上一个节点
                nnode=anode;//移到下一个节点
                //***蚂蚁未遇到食物***
                p+=1;
                if(p>=MM*MM)
                {break;}


            }


            //*****路径是否到达终点*****
            //路径到达终点，计算路径长度  最短路径？
            if((nnode.x==enode.x)||(nnode.y==enode.y))
            {
                pathlength[k][m]=one_path_length(path[k][m]);
                if(pathlength[k][m]<minkl)
                {
                    minkl=pathlength[k][m];  //最短路径的路径长度
                    mink=k;  //最短路径的迭代次数
                    minl=m;  //最短路径的蚂蚁代号
                }
            }
            //路径未到达终点，路径长度令为0
            else
            {pathlength[k][m]=0;break;}
        
        
        
        }


        //*****更新信息素*****
        //第k轮蚂蚁信息素增量
        for(var i=0;i<len(path[k]);i++)
        {
            for(var j=0;j<len(path[k][i]);j++)
            {
                Delta_PheromoneMatrix[path[k][i][j].x][path[k][i][j].y]+=Q/len(path[k][i]);
            }
        }
        //信息素挥发一部分，新增加一部分
        for(var i=0;i<MM;i++)
        {
            for(var j=0;j<MM;j++)
            {
                PheromoneMatrix[i][j]=(1-Rho)*PheromoneMatrix[i][j]+Delta_PheromoneMatrix[i][j];
            }
        }

    }
    return(path[mink][minl])
}
