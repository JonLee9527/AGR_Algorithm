#include "stdafx.h"

#include "blockManager.h"

#include "djiConst.h"
#include "PPETask.h"
#include "debugIO.h"
#include "djiUtils.h"
#include "djiConst.h"

#include <iostream>
using namespace std;

SprayLineseqBlock::SprayLineseqBlock(){ }

SprayLineseqBlock::SprayLineseqBlock(const SprayLineseqBlock&  _block)
{
	block = _block.block;
}

SprayLineseqBlock::SprayLineseqBlock(const vector<DjiPoint> & _block)
{
	for (int i = 1; i < _block.size();i=i+2)
	{
		block.push_back(DjiLineSeq(_block[i-1], _block[i]));
	}
}

unsigned SprayLineseqBlock::size() const
{
	return block.size();
}

DjiLineSeq& SprayLineseqBlock::operator[](const unsigned int index)
{
	return block[index];
}

const DjiLineSeq& SprayLineseqBlock::operator[](const unsigned int index) const
{
	return block[index];
}

DjiPoint SprayLineseqBlock::getPortPoint(int port)
{
	switch (port)
	{
	case 0:
		return block[0].s;
		break;
	case 1:
		return block[0].e;
		break;
	case 2:
		return block[block.size()-1].s;
		break;
	case 3:
		return block[block.size() - 1].e;
		break;
	default:
		printLog("error input in getPortPoint()", true);
		break;
	}
}

void SprayLineseqBlock::adjustBlockWithInport(int blockInport)
{
	vector<DjiLineSeq> temBlock = block;
	if (blockInport == 2 || blockInport == 3)
	{
		//倒序
		for (int i = 0; i < (int)block.size(); i++)
		{
			temBlock[i] = block[block.size() - 1 - i];
		}
	}

	if (blockInport == 3 || blockInport == 1)
	{
		for (int i = 0; i < temBlock.size(); i++)
		{
			temBlock[i].reverse();
		}
	}

	block = temBlock;
}


BlockManager::BlockManager()
{
	droneNum = 1;
}

BlockManager::BlockManager(vector<vector<DjiPoint > > _areaFrags,
	DjiPoint _homePoint, int _droneNum, const vector<DjiEdge* >& _allAroundObsEdges)
{
	if (_droneNum < 1)
	{
		printLog("inupt error droneNum<1 in BlockManager init", true);
	}


	blocks = vector<SprayLineseqBlock >(_areaFrags.size());
	for (int i = 0; i < _areaFrags.size(); i++)
	{
		blocks.push_back(SprayLineseqBlock(_areaFrags[i]));
	}
	droneNum = _droneNum;
	homePoint = _homePoint;

	for (int i = 0; i < _allAroundObsEdges.size();i++)
	{
		allAroundObsEdges.push_back(*_allAroundObsEdges[i]);
	}
}

unsigned BlockManager::size() const
{
	return blocks.size();
}
void BlockManager::erase(int index)
{
	blocks.erase(blocks.begin() + index);
}

void BlockManager::deleteEmptyBlock(vector<SprayLineseqBlock >& blocks)
{
	vector<SprayLineseqBlock > tem;
	for (int i = 0; i < blocks.size(); i++)
	{
		if (blocks[i].size()>0)
		{
			tem.push_back(blocks[i]);
		}
	}
	blocks = tem;
}

SprayLineseqBlock& BlockManager::operator[](const unsigned int index)
{
	return blocks[index];
}

const SprayLineseqBlock& BlockManager::operator[](const unsigned int index) const
{
	return blocks[index];
}

double BlockManager::getWholeDistance()
{
	double wholeDist = 0;
	for (int i = 0; i < blocks.size();i++)
	{
		for (int j = 0; j < blocks[i].size();j++)
		{
			wholeDist+=blocks[i][j].length();
		}
	}
	return wholeDist;
}

void BlockManager::findNearestPortInBlocks(DjiPoint startPoint, int & nearestBlockIndex, int& nearestPort)
{
	if (!blocks.size())
	{
		printLog("blocks.size()==0");
		return;
	}

	nearestBlockIndex = 0;
	nearestPort = 0;
	double nearestDist = findTwoPointsPath(allAroundObsEdges,startPoint, blocks[0].getPortPoint(0), vector<DjiPoint>());
	
	double temDist;

	for (int i = 0; i < blocks.size(); i++)
	{

		for (int j = 0; j < 4; j++)
		{
			temDist = findTwoPointsPath(allAroundObsEdges,startPoint, blocks[i].getPortPoint(j), vector<DjiPoint>());
			if (temDist < nearestDist)
			{
				nearestBlockIndex = i;
				nearestPort = j;
				nearestDist = temDist;
				
			}
		}
	}
}

void BlockManager::extractBlockPort0(int blockIndex, double& distaceLeft, vector<DjiPoint>& taskPoints)
{
	SprayLineseqBlock aBlock = blocks[blockIndex];

	int used = 0;
	for (used = 0; used < aBlock.size(); used++)
	{
		//drawImg(allAroundObsEdges[0].edge, COL_YELLOW_GREEN, CV_DRAW_LINE_SPRAY, 1);
		if (used % 2 == 0)
		{
			if (taskPoints.size()>0)  //从home点开始的到第一个喷洒点不采用寻路，默认home点到第一个喷洒点之间没有障碍物
			{
				findTwoPointsPath(allAroundObsEdges, taskPoints.back(), aBlock[used].s, taskPoints, false, false);
			}
			aBlock[used].s.s = POINT_SPRAY;
			aBlock[used].e.s = POINT_NO_SPRAY;
			taskPoints.push_back(aBlock[used].s);
			taskPoints.push_back(aBlock[used].e);
		}
		else
		{
			if (taskPoints.size() > 0)  //
			{
				findTwoPointsPath(allAroundObsEdges, taskPoints.back(), aBlock[used].s, taskPoints, false, false);
			}
			aBlock[used].e.s = POINT_SPRAY;
			aBlock[used].s.s = POINT_NO_SPRAY;
			taskPoints.push_back(aBlock[used].e);
			taskPoints.push_back(aBlock[used].s);
		}

		distaceLeft -= aBlock[used].length();

		if (distaceLeft < 0)
		{
			break;
		}
	}

	used++;
	if (used < blocks[blockIndex].size() && used>=0)
	{
		blocks[blockIndex].block = vector<DjiLineSeq>(aBlock.block.begin() + used, aBlock.block.end());
	}
	else
	{
		blocks.erase(blocks.begin() + blockIndex);
	}
}


vector<vector<DjiPoint> > BlockManager::calNearestPath()
{
	double wholedist = getWholeDistance();
	double oneTaskDist = wholedist / droneNum;
	if (oneTaskDist<=0)
	{
		printLog("oneTaskDist<=0", true);
	}

	int nearestBlockIndex;
	int inPort;
	DjiPoint startPoint = homePoint;
	double taskLeft = oneTaskDist;

	vector<vector<DjiPoint> > tasks;
	vector<DjiPoint> aMissionTask;

	while (true)
	{
		deleteEmptyBlock(blocks);//删除空的喷洒块

		if (blocks.size()==0)
		{
			break;
		}

		findNearestPortInBlocks(startPoint, nearestBlockIndex, inPort);

		blocks[nearestBlockIndex].adjustBlockWithInport(inPort);

		extractBlockPort0(nearestBlockIndex, taskLeft, aMissionTask);

		if (taskLeft <= 0 || blocks.size()==0)  //当前喷洒块留下一部分下一部飞机来执行
		{
			//该下一个飞机接受任务
			tasks.push_back(aMissionTask);

			aMissionTask.clear();

			taskLeft = oneTaskDist;
			startPoint = homePoint;
		}
		else  //当前喷洒块全部分给当前任务机，并继续分派其他块任务给当前飞机
		{

			if (aMissionTask.size() == 0)
			{
				printLog("aTask.size() == 0 in  calNearestPath()", true);
			}

			startPoint = aMissionTask.back();
		}
	}

	if (tasks.size()!=droneNum)
	{
		printLog("error: tasks.size()!=droneNum "+int2string(tasks.size()) + " != " + int2string(droneNum),true);
	}

	return tasks;
}



