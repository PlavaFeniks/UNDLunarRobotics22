class AStarNode;
int calculateDistance(int x, int y, AStarNode* targetNode);
int calculateDistance(int x, int y, int targetX, int targetY);

AStarNode ***mapOfPit = new AStarNode**[HEIGHT]; //each index is 10cm by 10cm
AStarNode* endNode;
AStarNode* startNode;
std::vector<AStarNode*>openNodes; //vector of input nodes

struct TransformationData//used for storing zed position
{
	float tx;
	float ty;
	float tz;
	
	//https://www.stereolabs.com/docs/api/classsl_1_1Rotation.html
	float rx;
	float ry;
	float rz;
} zedCurrent, zedGoal, zedNextGoal;

class AStarNode
{
	public:
	bool fake = true;
	float gCost = -1; //distance to starting node
	float hCost = -1; //distance to endNode
	float fCost = -1; //f = G + H
	int x,y,z; //position, y|x used for index
	
	//OccMapVals
	int Nobs = 0; //Number of times observed
	int OBS = 0; //Running value of observation
	double Pocc = 1.0; //Probability of occupancy
	int showOcc = 0;
	//PointCloud Processing
	float Zsum = 0;
	int Npoints = 0;
	float Zval = 0;
	float Disthere = 0;
	
	
	bool isTraversable = true;
	bool isClose = false;
	AStarNode* parent = NULL;
	AStarNode* child = NULL;
	
	AStarNode(int x, int y, int z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		fake = false;
	}
	
	void setXYZ(int x, int y, int z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
	void setGCost(int gCost, AStarNode* endNode)
	{
		this->gCost = gCost;
		hCost = calculateDistance(x, y, endNode);
		fCost = hCost + gCost;
	}
	
};

int calculateDistance(int x, int y, AStarNode* targetNode)
{
	int targetX = targetNode->x;
	int targetY = targetNode->y;
	float distance = sqrt(pow((targetX - x), 2) + pow((targetY - y), 2));
	return (int)(distance*10);
}

int calculateDistance(int x, int y, int targetX, int targetY)
{
	float distance = sqrt(pow((targetX - x), 2) + pow((targetY - y), 2));
	return (int)(distance*10);
}

void initializeTesselatedMap()
{
	//initialization of map with default values
	for (int i=0; i< HEIGHT; i++)
	{
		mapOfPit[i] = new AStarNode*[WIDTH];
		for (int j=0; j<WIDTH; j++)
		{
			mapOfPit[i][j] = new AStarNode(j, i, 0); //set 0 to height
		}
	}
}

void resetAStarTesselatedMap()
{
	//sets it up for another path to be plotted
	for (int i=0; i< HEIGHT; i++)
	{
		mapOfPit[i] = new AStarNode*[WIDTH];
		for (int j=0; j<WIDTH; j++)
		{
			mapOfPit[i][j]->isClose	= false; //set 0 to height
		}
	}
}


void definePath(AStarNode* currentNode) //define path from startNode to EndNode
{
	if (currentNode->parent == NULL)
	{
		if (currentNode == startNode) cout << "path found\n";
		else cout <<"path not found\n";
		return;
	}
	currentNode->parent->child = currentNode;
	definePath(currentNode->parent);
}
void FindPath(AStarNode* startingNode) //A* Algorithm, startNode and end node must be defined
{
	if (endNode->isTraversable == false) {cout << "great one\n";endNode->isTraversable=true;}
	openNodes.push_back(startNode);
	AStarNode* current = NULL;
	int index = 0;
	while(openNodes.size() != 0)
	{
		current = NULL;
		//sets current to node in openNodes with lowest fCost
		for (int i=0; i<(int)openNodes.size(); i++)
		{
			if (current == NULL) {current = openNodes[i]; index = i;}
			else if (openNodes[i]->fCost < current->fCost){current = openNodes[i]; index = i;}
		}
		if (current->x == endNode->x and current->y == endNode->y) break;
		current->isClose = true;
		
		AStarNode** neighbors = new AStarNode*[8];
		//for (int i=0; i<8; i++) {neighbors[i] = NULL;}
		int x = current->x;
		int y = current->y;
		
		int counter = 0;
		for (int i=-1; i<=1; i++)
		{
			for (int j=-1; j<=1; j++)
			{
				int newX = x+i;
				int newY = y+j;
				if ((newY==0 and newX==0) or newY<0 or newX<0 or newY>HEIGHT-1 or newX>WIDTH-1) continue;
				neighbors[counter] = mapOfPit[newY][newX];
				counter++;
			}
		}
		
		for (int i=0; i<8; i++)
		{
			if (counter-1 < i) break;
			
			AStarNode* neighbor = (AStarNode*)neighbors[i];
			if (neighbor == NULL) break; //go to next neighbor if current one doesnt exist
			if (neighbor->isTraversable == false or neighbor->isClose == true or neighbor->fake == true) continue; //go to next neighbor if current neighbor is not traversable or closed

		
			int gCost = current->gCost + calculateDistance(current->x, current->y, neighbor->x, neighbor->y); //add gCost from parent and get distance from child to parent
		
			if (neighbor->parent == NULL)
			{
				neighbor->parent = current;
				neighbor->setGCost(gCost, endNode);
				openNodes.push_back(neighbor);
			}
			else if (neighbor->gCost > gCost)
			{
				neighbor->parent = current;
				neighbor->setGCost(gCost, endNode);
			}
			
		}
		openNodes.erase(openNodes.begin() + index);
		
			
		}
	cout << "a* complete\n";
	definePath(endNode);
}


