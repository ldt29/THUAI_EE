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
extern const THUAI5::SoftwareType playerSoftware = THUAI5::SoftwareType::PowerEmission;

// 选手被动技能，选手 !!必须!! 定义此变量来选择被动技能
extern const THUAI5::HardwareType playerHardware = THUAI5::HardwareType::EnergyConvert;

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
std::shared_ptr<const THUAI5::Prop> uploadprop(IAPI& api,std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props);
double getDtoRobot(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Robot> other);
double getDtoProp(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Prop> prop);
std::vector<node> dijkstra(int x, int y, int sx, int sy);
void selfControl(std::shared_ptr<const THUAI5::Robot> self, IAPI& api);
void moveToProp(std::shared_ptr<const THUAI5::Prop> prop, IAPI& api);
double getDirection(uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY);
bool search(std::shared_ptr<const THUAI5::Robot> self, uint32_t selfPoisitionX, uint32_t selfPoisitionY, uint32_t aimPositionX, uint32_t aimPositionY);
bool evade(std::shared_ptr<const THUAI5::Robot> self,IAPI& api);
bool ismove(THUAI5::PlaceType type);

//判断某地是否可以移动
bool ismove(THUAI5::PlaceType Type) {
	int type = int(Type);
	if (type == 1 || (type > 4 && type < 13)) {
		return false;
	}
	return true;
}

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
double getDtoRobot(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::Robot> other) {
	auto selfx = self->x;
	auto selfy = self->y;
	auto otherx = other->x;
	auto othery = other->y;
	return sqrt((selfx - otherx) * (selfx - otherx) + (selfy - othery) * (selfy - othery));
}

//得到自身到干扰弹的距离
double getDtoJammer(std::shared_ptr<const THUAI5::Robot> self, std::shared_ptr<const THUAI5::SignalJammer> jammer) {
	auto selfx = self->x;
	auto selfy = self->y;
	auto otherx = jammer->x;
	auto othery = jammer->y;
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
std::shared_ptr<const THUAI5::Prop> uploadcpu(std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props) {
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
std::shared_ptr<const THUAI5::Prop> uploadprop(IAPI& api, std::shared_ptr<const THUAI5::Robot> self, std::vector<std::shared_ptr<const THUAI5::Prop>> props) {
	double min = 1e6;
	std::shared_ptr<const THUAI5::Prop> ans = nullptr;
	for (auto prop : props) {
		if (getDtoProp(self, prop) < min) {
			int curmin = getDtoProp(self, prop);
			auto robots = api.GetRobots();
			int flag = 0;
			for (auto robot : robots) {
				if (robot->teamID != self->teamID) {
					continue;
				}
				if (getDtoProp(robot, prop) < curmin-1.0) {
					if(getDtoRobot(self, robot)<2000|| getDtoProp(robot, prop)<curmin/2.0)
						flag = 1;
				}
			}
			if (flag == 1)continue;
			ans = prop;
			min = curmin;
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
		if (the.y + 1 < 50 && isWall[the.x][the.y + 1] != 1 && (isWall[the.x][the.y + 1] < 5 || isWall[the.x][the.y + 1] == 13) && !flag[the.x][the.y + 1]) {
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
void selfControl(std::shared_ptr<const THUAI5::Robot> self, IAPI& api) {
	int propx = api.CellToGrid(api.GridToCell(self->x));
	int propy = api.CellToGrid(api.GridToCell(self->y));
	int cellx = api.GridToCell(self->x);
	int celly = api.GridToCell(self->y);
	/*if (propx < self->x) {
		api.MoveUp(1000 * (self->x - propx) / self->speed);
		
	}
	else {
		api.MoveDown(1000 * (propx - self->x) / self->speed);
		
	}
	if (propy < self->y) {
		api.MoveLeft(1000 * (self->y - propy) / self->speed);
		
	}
	else {
		api.MoveRight(1000 * (propy - self->y) / self->speed);
		
	}*/
	auto type1 = api.GetPlaceType(cellx - 1, celly - 1);
	auto type2 = api.GetPlaceType(cellx + 1, celly - 1);
	auto type3 = api.GetPlaceType(cellx - 1, celly + 1);
	auto type4 = api.GetPlaceType(cellx + 1, celly + 1);
	auto type5 = api.GetPlaceType(cellx, celly + 1);
	auto type6 = api.GetPlaceType(cellx, celly - 1);
	auto type7 = api.GetPlaceType(cellx + 1, celly);
	auto type8 = api.GetPlaceType(cellx - 1, celly);
	if (angle > 2 * PI - 0.1 || angle < 0.1 || (angle<PI + 0.1 && angle > PI - 0.1)) {
		if (!ismove(type1) && ismove(type2) && ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveRight(100);

		}
		if (ismove(type1) && !ismove(type2) && ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveRight(100);		

		}
		if (ismove(type1) && ismove(type2) && !ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveLeft(100);
			
		}
		if (ismove(type1) && ismove(type2) && ismove(type3) && !ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveLeft(100);
			
		}
	}
	else {
		
		if (ismove(type1) && ismove(type2) && !ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveDown(100);
			
		}
		
		if (!ismove(type1) && ismove(type2) && ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveDown(100);;
			
		}
		
		if (ismove(type1) && ismove(type2) && ismove(type3) && !ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveUp(100);
			
		}
		
		if (ismove(type1) && !ismove(type2) && ismove(type3) && ismove(type4) && ismove(type5) && ismove(type6) && ismove(type7) && ismove(type8)) {
			api.MoveUp(100);
			
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
	std::vector<node> L = dijkstra(selfx, selfy, propx, propy);
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

//躲闪
bool evade(std::shared_ptr<const THUAI5::Robot> self, IAPI& api) {
	bool flag = false;
	auto jammers = api.GetSignalJammers();
	double minDis = 9000;
	std::shared_ptr<const THUAI5::SignalJammer> Mjammer = nullptr;
	for (auto jammer : jammers) {
		if (jammer->parentTeamID == self->teamID) {
			continue;
		}
		int dis = getDtoJammer(self, jammer);
		if (dis < minDis) {
			minDis = dis;
			Mjammer = jammer;
		}
	}
	if (Mjammer != nullptr) {
		int e = getDirection(Mjammer->x, Mjammer->y, self->x, self->y);
		if (abs(Mjammer->facingDirection - e) < 0.5) {
			api.MovePlayer(1000000 / self->speed, e - PI / 2);
			api.MovePlayer(2000000 / self->speed, e);
			flag = true;
		}
	}
	return flag;
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
					api.Attack(e + 0.2);
					api.Attack(e - 0.2);

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
	
	//使用cpu
	if (self->cpuNum > 2){
		api.UseCPU(self->cpuNum);
	}

	api.Wait();
	std::cout << self->isResetting;

	attackaround(api, self);
	api.UseCommonSkill();


	//随机攻击
	if (self->signalJammerNum > 3) {
		api.Attack(getDirection(self->x, self->y, 25 * 1000, 25 * 1000));
	}

	//躲闪
	if (evade(self, api)) {
		return;
	}

	//调整
	selfControl(self, api);
	//获取场上的道具信息
	auto props = api.GetProps();
	auto cpu = THUAI5::PropType::CPU;
	//最近的cpu
	/*auto tocpu = uploadcpu(self, props);
	if (tocpu != nullptr) {
		//捡cpu
		api.Pick(cpu);
		moveToProp(tocpu, api);
		return;
	}*/
	//最近的prop
	auto toprop = uploadprop(api ,self, props);
	if (toprop != nullptr) {
		if (api.Pick(toprop->type))
			api.UseProp();
		moveToProp(toprop, api);
	}
}
