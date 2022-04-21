#include <random>
#include "../include/AI.h"
#include<queue>
#include<cmath>
#include<algorithm>
#pragma warning(disable:C26495)

/* 请于 VS2019 项目属性中开启 C++17 标准：/std:c++17 */

// 为假则play()调用期间游戏状态更新阻塞，为真则只保证当前游戏状态不会被状态更新函数与IAPI的方法同时访问
extern const bool asynchronous = true;


// 选手主动技能，选手 !!必须!! 定义此变量来选择主动技能
extern const THUAI5::SoftwareType playerSoftware = THUAI5::SoftwareType::Amplification;

// 选手被动技能，选手 !!必须!! 定义此变量来选择被动技能
extern const THUAI5::HardwareType playerHardware = THUAI5::HardwareType::PowerBank;

namespace
{
	[[maybe_unused]] std::uniform_real_distribution<double> direction(0, 2 * 3.1415926);
	[[maybe_unused]] std::default_random_engine e{ std::random_device{}() };
}

const double PI = 3.1415926;
//移动方向
double angle = 0;
//探图节点
struct node {
	int x;
	int y;
	int prex;
	int prey;
};

class Enemy
{
public:
	double getnextx()
	{
		return 2 * x - prex;
	}
	double getnexty()
	{
		return 2 * y - prey;
	}
	double x;//这一次的坐标
	double y;
	double prex;//上一次的坐标 确定一条直线
	double prey;
	double nextx;//预测接下来的坐标 假设敌人速度不变
	double nexty;
	double t;
	double speed;//敌人速度
	double angle;//敌人移动角度
}enemy[4];

//场上其他人的位置信息
std::set<node> PofOtherRobot;
void isWalling(IAPI& api);
std::shared_ptr<const THUAI5::Prop> uploadcpu(std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props);
std::shared_ptr<const THUAI5::Prop> uploadprop(std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props);
double getDtoRobot(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Robot> other);
double getDtoProp(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Prop> prop);
std::vector<node> dijkstra(int x, int y, int sx, int sy);
void selfControl(std::shared_ptr<const THUAI5::Robot> self,IAPI& api);
void moveToProp( std::shared_ptr<const THUAI5::Prop> prop, IAPI& api);
double getDirection(uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY);
bool search(std::shared_ptr<const THUAI5::Robot> self, uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY);

//获取角度
double getDirection(uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY)
{
	double delta_x = double(aimPositionX) - double(selfPoisitionX);
	double delta_y = double(aimPositionY) - double(selfPoisitionY);
	double direction = atan2(delta_y, delta_x);
	if (direction < 0)
	{
		direction = 2 * PI + direction;
	}
	return direction;
}

//攻击距离
bool search(std::shared_ptr<const THUAI5::Robot> self, uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY)
{
	double SEARCH_R = self->attackRange + 1000;
	return SEARCH_R > sqrt((aimPositionX - selfPoisitionX) * (aimPositionX - selfPoisitionX) + (aimPositionY - selfPoisitionY) * (aimPositionY - selfPoisitionY));
}

//得到自身到道具的距离
double getDtoProp(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Prop> prop) {
	auto selfx = self->x;
	auto selfy = self->y;
	auto propx = prop->x;
	auto propy = prop->y;
	return sqrt((selfx - propx) * (selfx - propx) + (selfy - propy) * (selfy - propy));
}

//得到自身到机器人的距离
double getDtoRobot(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Prop> other) {
	auto selfx = self->x;
	auto selfy = self->y;
	auto otherx = other->x;
	auto othery = other->y;
	return sqrt((selfx - otherx) * (selfx - otherx) + (selfy - othery) * (selfy - othery));	
}


//保存探图结果
static std::vector<std::vector<int> > isWall(50, std::vector<int>(50, 0));
//判断探图是否结束
static bool isWalled = false;
//探图
void isWalling(IAPI& api) {
	for (int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++) {
			isWall[i][j] = (int)api.GetPlaceType(i, j);
		}
	}
	isWalled = true;
}

//记录目前场上的cpu
std::vector<std::shared_ptr<const THUAI5::Prop>> cpus;
//更新场上cpu数据并返回距离自身最近的cpu指针,否则返回空指针
std::shared_ptr<const THUAI5::Prop> uploadcpu(std::shared_ptr<const THUAI5::Robot> self,std::vector<std::shared_ptr<const THUAI5::Prop>> props) {
	std::vector<std::shared_ptr<const THUAI5::Prop>>().swap(cpus);
	for (auto prop : props)
	{
		if (prop->type == THUAI5::PropType::CPU) {
			cpus.push_back(prop);
		}
	}
	double min = 1e6;
	std::shared_ptr<const THUAI5::Prop> ans = nullptr;
	for (auto cpu : cpus) {
		if (getDtoProp(self, cpu) < min) {
			ans = cpu;
			min = getDtoProp(self, cpu);
		}
	}
	return ans;
}


//更新场上prop数据并返回距离自身最近的prop指针,否则返回空指针
std::shared_ptr<const THUAI5::Prop> uploadprop(std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props) {
	double min = 1e6;
	std::shared_ptr<const THUAI5::Prop> ans = nullptr;
	for (auto prop : props) {
		if (getDtoProp(self, prop) < min) {
			ans = prop;
			min = getDtoProp(self, prop);
		}
	}
	return ans;
}


//路线
std::vector<node> dijkstra(int x, int y, int sx, int sy) {
	std::vector<node> ans;
	if (x == sx && y == sy)return ans;
	std::vector<std::vector<node>> Node(50, std::vector<node>(50));
	std::vector<std::vector<int>> flag(50, std::vector<int>(50, 0));
	std::queue<node> q;
	q.push({ x,y,0,0 });
	flag[x][y] = 1;
	while (q.size()) {
		node the = q.front();
		q.pop();
		if (the.x + 1 < 50 && isWall[the.x + 1][the.y] != 1 && (isWall[the.x + 1][the.y] < 5 || isWall[the.x + 1][the.y] == 13) && !flag[the.x + 1][the.y]) {
			Node[the.x + 1][the.y].x = the.x + 1;
			Node[the.x + 1][the.y].y = the.y;
			Node[the.x + 1][the.y].prex = the.x;
			Node[the.x + 1][the.y].prey = the.y;
			flag[the.x + 1][the.y] = 1;
			q.push(Node[the.x + 1][the.y]);
		}
		if (the.x - 1 >= 0 && isWall[the.x - 1][the.y] != 1 && (isWall[the.x - 1][the.y] < 5 || isWall[the.x - 1][the.y] == 13) && !flag[the.x - 1][the.y]) {
			Node[the.x - 1][the.y].x = the.x - 1;
			Node[the.x - 1][the.y].y = the.y;
			Node[the.x - 1][the.y].prex = the.x;
			Node[the.x - 1][the.y].prey = the.y;
			flag[the.x - 1][the.y] = 1;
			q.push(Node[the.x - 1][the.y]);
		}
		if (the.y + 1 < 50 && isWall[the.x][the.y + 1] != 1 && (isWall[the.x][the.y + 1]<5 || isWall[the.x][the.y + 1] == 13)  && !flag[the.x][the.y + 1]) {
			Node[the.x][the.y + 1].x = the.x;
			Node[the.x][the.y + 1].y = the.y + 1;
			Node[the.x][the.y + 1].prex = the.x;
			Node[the.x][the.y + 1].prey = the.y;
			flag[the.x][the.y + 1] = 1;
			q.push(Node[the.x][the.y + 1]);
		}
		if (the.y - 1 >= 0 && isWall[the.x][the.y - 1] != 1 && (isWall[the.x][the.y - 1] < 5 || isWall[the.x][the.y - 1] == 13) && !flag[the.x][the.y - 1]) {
			Node[the.x][the.y - 1].x = the.x;
			Node[the.x][the.y - 1].y = the.y - 1;
			Node[the.x][the.y - 1].prex = the.x;
			Node[the.x][the.y - 1].prey = the.y;
			flag[the.x][the.y - 1] = 1;
			q.push(Node[the.x][the.y - 1]);
		}
	}
	node temp = Node[sx][sy];
	ans.push_back(temp);
	while (temp.prex != x || temp.prey != y) {
		temp = Node[temp.prex][temp.prey];
		ans.push_back(temp);
	}
	reverse(ans.begin(), ans.end());
	return ans;
}
//调整自身位置
void selfControl(std::shared_ptr<const THUAI5::Robot> self,IAPI& api) {
	int propx = api.CellToGrid(api.GridToCell(self->x));
	int propy = api.CellToGrid(api.GridToCell(self->y));

	if (propx < self->x) {
		api.MoveUp(1000 * (self->x - propx) / self->speed);
		angle = PI;
	}
	else {
		api.MoveDown(1000 * (propx - self->x) / self->speed);
		angle = 0;
	}
	if (propy < self->y) {
		api.MoveLeft(1000 * (self->y - propy) / self->speed);
		angle = PI * 1.5;
	}
	else {
		api.MoveRight(1000 * (propy - self->y) / self->speed);
		angle = PI * 0.5;
	}

	if (angle==0||angle==PI) {
		int type = int(api.GetPlaceType(propx - 1, propy-1));
		if (type == 1||(type>4&&type<13)) {
			api.MoveRight(5);
			angle = PI*0.5;
		}
		type = int(api.GetPlaceType(propx + 1, propy-1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveRight(5);
			angle = PI*0.5;
		}
		type = int(api.GetPlaceType(propx - 1, propy + 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveLeft(5);
			angle = PI*1.5;
		}
		type = int(api.GetPlaceType(propx + 1, propy + 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveLeft(5);
			angle = PI*1.5;
		}
	}
	else {
		int type = int(api.GetPlaceType(propx-1, propy + 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveDown(5);
			angle = 0;
		}
		type = int(api.GetPlaceType(propx-1, propy - 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveDown(5);
			angle = 0;
		}
		type = int(api.GetPlaceType(propx + 1, propy + 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveUp(5);
			angle = PI;
		}
		type = int(api.GetPlaceType(propx + 1, propy - 1));
		if (type == 1 || (type > 4 && type < 13)) {
			api.MoveUp(5);
			angle = PI;
		}
	}
}

//向道具移动
void moveToProp(std::shared_ptr<const THUAI5::Prop> prop, IAPI& api) {
	auto self = api.GetSelfInfo();
	int selfx = api.GridToCell(self->x);
	int selfy = api.GridToCell(self->y);
	int propx = api.GridToCell(prop->x);
	int propy = api.GridToCell(prop->y);
	std::vector<node> L = dijkstra(selfx,selfy,propx,propy);
	if (L.size()) {
		auto nd = L[0];
		if (nd.x < selfx) {
			api.MoveUp(1000000 / self->speed);
			angle = PI;
		}
		else if (nd.x > selfx) {
			api.MoveDown(1000000 / self->speed);
			angle = 0;
		}
		else if (nd.y < selfy) {
			api.MoveLeft(1000000 / self->speed);
			angle = PI * 1.5;
		}
		else if (nd.y > selfy) {
			api.MoveRight(1000000 / self->speed);
			angle = PI * 0.5;
		}
	}

}

//判断是否攻击
bool Attackornot(std::shared_ptr<const THUAI5::Robot> self, uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY)
{
	double ATTACK_R = self->attackRange;
	return ATTACK_R > sqrt((aimPositionX - selfPoisitionX) * (aimPositionX - selfPoisitionX) + (aimPositionY - selfPoisitionY) * (aimPositionY - selfPoisitionY));
}

//攻击
void attackaround(IAPI& api, std::shared_ptr<const THUAI5::Robot> self)
{
	auto player = api.GetRobots();
	if (!player.empty() && self->signalJammerNum > 0)//
	{
		for (int i = 0; i < player.size(); i++)
		{

			if (self->teamID != player[i]->teamID && !player[i]->isResetting && search(self, self->x, self->y, player[i]->x, player[i]->y))//进入预定范围
			{
				enemy[i].prex = enemy[i].x;//刷新坐标
				enemy[i].prey = enemy[i].y;
				enemy[i].x = player[i]->x;
				enemy[i].y = player[i]->y;


				if (Attackornot(self, self->x, self->y, enemy[i].getnextx(), enemy[i].getnexty()))//在攻击范围内
				{
					double e = getDirection(self->x, self->y, enemy[i].getnextx(), enemy[i].getnexty());//定角度
					api.Attack(e);
					api.Attack(e);
					api.Attack(e);
					api.Attack(e);
					api.Attack(e);
				}
			}
		}
	}
};

void AI::play(IAPI& api)
{
	std::ios::sync_with_stdio(false);
	if (!isWalled) {
		isWalling(api);
	}
	//得到个人信息
	auto self = api.GetSelfInfo();
	//调整
	selfControl(self, api);
	//获取场上的道具信息
	auto props = api.GetProps();
	auto cpu = THUAI5::PropType::CPU;
	//最近的cpu
	auto tocpu = uploadcpu(self, props);
	if (tocpu != nullptr) {
		//捡cpu
		api.Pick(cpu);
		moveToProp(tocpu,api);
		return;
	}
	//最近的prop
	auto toprop = uploadprop(self, props);
	if (toprop != nullptr) {
		if(api.Pick(toprop->type))
			api.UseProp();
		moveToProp(toprop, api);
	}
	api.Wait();
	std::cout << self->isResetting;

	attackaround(api, self);
	api.UseCommonSkill();
}
