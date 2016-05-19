#ifndef DJI_ALG_AARICULTURE_BLOCK_MANAGER
#define DJI_ALG_AARICULTURE_BLOCK_MANAGER

#include "djiPoint.h"

#include "DjiEdge.h"

#include "SprayTask.h"

/*SprayBlock 四个port 0 1 2 3 定义
* vector<DjiLineSeq>   
*    0   ->  n
*
*e   1.......3    
*s   0.......2
*/

class SprayLineseqBlock
{
public:
	vector<DjiLineSeq> block;
	SprayLineseqBlock();
	SprayLineseqBlock(const SprayLineseqBlock&  _block);
	SprayLineseqBlock(const vector<DjiPoint > & _block);

	unsigned size() const;

	void clear(){ 
		block.clear(); }
	void push_back(DjiLineSeq seq){
		block.push_back(seq);
	}

	DjiLineSeq& operator[](const unsigned int index);

	const DjiLineSeq& operator[](const unsigned int index) const;

	void adjustBlockWithInport(int inport);

	DjiPoint getPortPoint(int port);

	double nearPort(DjiPoint startPoint, int& nearport);

};

class BlockManager
{
private:
	//init data
	vector<SprayLineseqBlock > blocks;
	vector<DjiEdge > allAroundObsEdges;
	int droneNum;
	DjiPoint homePoint;

public:
	BlockManager();

	BlockManager(vector<vector<DjiPoint > > _areaFrags, DjiPoint _homePoint, int _droneNum, const vector<DjiEdge* >& _allAroundObsEdges);

	//贪心算法
	vector<vector<DjiPoint> > calNearestPath();
private:

	unsigned size() const;

	void erase(int index);

	SprayLineseqBlock& operator[](const unsigned int index);

	const SprayLineseqBlock& operator[](const unsigned int index) const;

	void deleteEmptyBlock(vector<SprayLineseqBlock >& blocks);

	double getWholeDistance();

	void findNearestPortInBlocks(DjiPoint startPoint, int & startIndex, int& inPort);

	void extractBlockPort0(int blockIndex, double& distaceLeft, vector<DjiPoint>& task);
};



#endif

