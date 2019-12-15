#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include<vector>
#include<algorithm>

using namespace std;

class pathFinder : public olc::PixelGameEngine
{
private:
	struct Node {
		bool visited = false;
		bool obstacle = false;
		float globalGoal;
		float localGoal;
		int x;		// x and y are used to represent nodes in space
		int y;
		vector <Node*> neighbours;
		Node* parent;
	};
	Node* nodes = NULL;
	int mapHeight = 16;
	int mapWidth = 16;
	Node* startNode = NULL;
	Node* endNode = NULL;
public:
	pathFinder()
	{
		sAppName = "Path finder - Astar";
	}

public:
	bool OnUserCreate() override
	{
		nodes = new Node[mapHeight * mapWidth];
		for (int i = 0; i < mapWidth; i++) {
			for (int j = 0; j < mapHeight; j++) {
				nodes[j * mapWidth + i].x = i;
				nodes[j * mapWidth + i].y = j;
				nodes[j * mapWidth + i].visited = false;
				nodes[j * mapWidth + i].obstacle = false;
				nodes[j * mapWidth + i].parent = NULL;
			}
		}
		for (int i = 0; i < mapWidth;i++) {
			for (int j = 0; j < mapHeight;j++) {
				if (j > 0)
					nodes[j * mapWidth + i].neighbours.push_back(&nodes[(j - 1) * mapWidth + i]);
				if(j<mapHeight-1)
					nodes[j * mapWidth + i].neighbours.push_back(&nodes[(j + 1) * mapWidth + i]);
				if(i>0)
					nodes[j * mapWidth + i].neighbours.push_back(&nodes[j * mapWidth + (i-1)]);
				if(i<mapWidth-1)
					nodes[j * mapWidth + i].neighbours.push_back(&nodes[j * mapWidth + (i+1)]);
			}
		}
		//startNode = &nodes[4 + mapWidth + 4];
		//endNode = &nodes[8 + mapWidth + 8];
		return true;
	}


	auto heuristic(Node* a, Node* b)
	{
		return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
	}

	void A_starSolve() {
		for (int i = 0; i < mapWidth; i++) {
			for (int j = 0; j < mapHeight; j++) {
				nodes[j * mapWidth + i].globalGoal = INFINITY;
				nodes[j * mapWidth + i].localGoal = INFINITY;
				nodes[j * mapWidth + i].parent = NULL;
				nodes[j * mapWidth + i].visited = false;
			}
		}






		Node* currentNode = startNode;
		startNode->globalGoal = heuristic(startNode, endNode);
		startNode->localGoal = 0.0f;

		list<Node*> notVisited;
		notVisited.push_back(startNode);

		while (!notVisited.empty() && currentNode!=endNode) {
			notVisited.sort([](const Node* A, const Node* B) {return A->globalGoal < B->globalGoal; });
			while (!notVisited.empty() && notVisited.front()->visited)
			{
				notVisited.pop_front();
			}
			if (notVisited.empty()) break;
			currentNode = notVisited.front();
			currentNode->visited = true;

			for (auto currentNeighbour : currentNode->neighbours) {
				if (!currentNeighbour->visited && !currentNeighbour->obstacle)
					notVisited.push_back(currentNeighbour);

				float minLG = currentNode->localGoal + heuristic(currentNeighbour, currentNode);
				if (minLG < currentNeighbour->localGoal) {
					currentNeighbour->parent = currentNode;
					currentNeighbour->localGoal = minLG;
					currentNeighbour->globalGoal = currentNeighbour->localGoal + heuristic(currentNeighbour, endNode);
				}
			}
		}
	}





	bool OnUserUpdate(float fElapsedTime) override
	{
		// called once per frame
		int nodeSize = 9;
		int nodeBorder = 2;

		//Get mouse co-ordinates
		int selectedNodeX = GetMouseX() / nodeSize;
		int selectedNodeY = GetMouseY() / nodeSize;

		if (GetMouse(0).bReleased) {
			if(selectedNodeX >0 && selectedNodeX<=mapHeight)
				if (selectedNodeY > 0 && selectedNodeY <= mapWidth) {
					if (GetKey(olc::Key::CTRL).bHeld) {
						startNode = &nodes[selectedNodeY * mapWidth + selectedNodeX];
					}
					else if (GetKey(olc::Key::SHIFT).bHeld) {
						endNode = &nodes[selectedNodeY * mapWidth + selectedNodeX];
					}
					else
					nodes[selectedNodeY * mapWidth + selectedNodeX].obstacle = !nodes[selectedNodeY * mapWidth + selectedNodeX].obstacle;
					if(startNode!=NULL && endNode!=NULL)
						A_starSolve();
				}
		}
		
		FillRect(0, 0, ScreenWidth(), ScreenHeight(), olc::BLACK);
		for (int i = 0; i < mapWidth; i++) {
			for (int j = 0; j < mapHeight; j++) {
				for (auto n : nodes[j * mapWidth + i].neighbours) {
					DrawLine(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2, n->x * nodeSize + nodeSize / 2, n->y * nodeSize + nodeSize / 2);
				}
			}
		}
		for (int i = 0; i < mapWidth;i++) {
			for (int j = 0; j < mapHeight;j++) {
				bool flag = nodes[j * mapWidth + i].obstacle;
				FillRect(i * nodeSize + nodeBorder, j * nodeSize + nodeBorder, nodeSize - (2 * nodeBorder), nodeSize - (2 * nodeBorder), flag ? olc::DARK_GREY : olc::RED);
				if(&nodes[j * mapWidth + i] == startNode)
					FillRect(i * nodeSize + nodeBorder, j * nodeSize + nodeBorder, nodeSize - (2 * nodeBorder), nodeSize - (2 * nodeBorder), olc::GREEN);
				if (&nodes[j * mapWidth + i] == endNode)
					FillRect(i * nodeSize + nodeBorder, j * nodeSize + nodeBorder, nodeSize - (2 * nodeBorder), nodeSize - (2 * nodeBorder), olc::YELLOW);
				if(nodes[j*mapWidth +i].visited)
					FillRect(i * nodeSize + nodeBorder, j * nodeSize + nodeBorder, nodeSize - (2 * nodeBorder), nodeSize - (2 * nodeBorder),  olc::BLUE);
				//FillRect(i * nodeSize + nodeBorder, j * nodeSize + nodeBorder, nodeSize - (2*nodeBorder), nodeSize - (2*nodeBorder), flag ? olc::DARK_GREY : olc::RED);
			}
		}

		if (endNode != NULL) {
			Node* temp = endNode;
			while (temp->parent != NULL) {
				DrawLine(temp->x * nodeSize + nodeSize / 2, temp->y * nodeSize + nodeSize / 2, temp->parent->x * nodeSize + nodeSize / 2, temp->parent->y * nodeSize + nodeSize / 2 , olc::YELLOW);
				temp = temp->parent;
			}
		}


		return true;
	}

};


int main()
{
	pathFinder app;
	if (app.Construct(160, 160, 6, 6))
		app.Start();

	return 0;
}