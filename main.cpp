/* Reference for MinCut MaxFlow Algorithm : "https://www.geeksforgeeks.org/ford-fulkerson-algorithm-for-maximum-flow-problem/" */
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


class Edge {
public:
	int weight;

	int fromVertex;
	int toVertex;
	Edge(int from, int to, int weight) {
		(*this).weight = weight;
		(*this).toVertex = to;
		(*this).fromVertex = from;
	}


};
class Vertex {
public:
	 vector<Edge> edges;
	int vertex;
	bool markwhite;
	
	void edgeInsertion(Edge edge) {
		(*this).edges.push_back(edge);
	}

	Vertex() {
		(*this).vertex = -1;
		markwhite = false;
	}
};

bool breadthSearch( vector<Vertex> &graph, int source, int sink,  vector<int> &parent,int pixels) {
    vector<int> visitedArray(pixels+2,0);
	visitedArray[source] = 1;
	queue <Vertex> toBeVisited;
	toBeVisited.push(graph[source]);
	parent[source] = -1;
	while (!toBeVisited.empty()) {
		Vertex visitingNode = toBeVisited.front();
		toBeVisited.pop();
		vector<Edge> alledges = visitingNode.edges;
		
		for (unsigned int i = 0; i < visitingNode.edges.size();i++) {
			if (visitingNode.edges[i].weight > 0 && visitedArray[visitingNode.edges[i].toVertex]==0) {
				toBeVisited.push(graph[visitingNode.edges[i].toVertex]);
				visitedArray[visitingNode.edges[i].toVertex] = 1;
				parent[visitingNode.edges[i].toVertex] = visitingNode.vertex;
				if (visitingNode.edges[i].toVertex == sink) {
					return true;
				}
			}
		}
	}
	return (visitedArray[sink] == 1);
}

void breadthSearchforColoring(vector<Vertex> &graph, int source,  int pixels,Mat &out_image,int width) {

	vector<int> visitedArray(pixels+2, 0);
	visitedArray[source] = 1;
	queue <Vertex> toBeVisited;
	toBeVisited.push(graph[source]);
	int ik = graph[source].edges[0].toVertex/width;
	int jk = graph[source].edges[0].toVertex%width;
		
	while (!toBeVisited.empty()) {
		Vertex visitingNode = toBeVisited.front();
		toBeVisited.pop();
		vector<Edge> alledges = visitingNode.edges;
		
		
		for (unsigned int i = 0; i < visitingNode.edges.size(); i++) {
			if (visitingNode.edges[i].weight > 0 && visitedArray[visitingNode.edges[i].toVertex] == 0) {
				toBeVisited.push(graph[visitingNode.edges[i].toVertex]);
				visitedArray[visitingNode.edges[i].toVertex] = 1;
				int pix = visitingNode.edges[i].toVertex;
				graph[visitingNode.vertex].markwhite = true;
				int i = pix/width;
				int j = pix%width;
				Vec3b pixel = out_image.at<Vec3b>(i,j);
				pixel[0] = 255;
				pixel[1] = 255;
				pixel[2] = 255;
				out_image.at<Vec3b>(i, j) = pixel;
			}
		}
	}
}
void mincutmaxflow( vector<Vertex> &graph, int source, int sink,const int pixels,Mat &out_image,int width) {
	 vector<int> parent(pixels+2, 0);
	 int maximumflow = 0;
	 int minimumFlow = INT16_MAX;
	
	 while (breadthSearch(graph, source, sink, parent,pixels)) {
		minimumFlow = INT16_MAX;
		int edgeweight = 0;
		for (int j = sink; j != source; j = parent[j]) {
			for (unsigned int i = 0; i < graph[parent[j]].edges.size(); i++) {
				vector<Edge> edges = graph[parent[j]].edges;
				if (graph[parent[j]].edges[i].toVertex == j) {
					minimumFlow = std::min(minimumFlow, graph[parent[j]].edges[i].weight);
				}
			}
		}
	

		for (int j = sink; j != source; j = parent[j]) {
			for (unsigned int i = 0; i < graph[parent[j]].edges.size(); i++) {
				if (graph[parent[j]].edges[i].toVertex == j) {
					graph[parent[j]].edges[i].weight = graph[parent[j]].edges[i].weight - minimumFlow;
				}
			}
			for (unsigned int i = 0; i < graph[j].edges.size(); i++) {
				if (graph[j].edges[i].toVertex == parent[j]){
					graph[j].edges[i].weight = graph[parent[j]].edges[i].weight + minimumFlow;
				}
			}
			
		}
		maximumflow = maximumflow + minimumFlow;
	}
	breadthSearchforColoring(graph, source,pixels,out_image,width);
}

int maximum (int a,int b,int c)
{    
if(a>b){
	if(a>c){
	return a;
	       }
	else{
	return c;
	    } 
       }
else if(b>c){
	return b;
            }
else{
        return c;
}

}
int maxfour(int a,int b,int c,int d)
{
    int max;
    max = (a > b? a : b);  
    /* Find the greater number in c and d, and compare to the previously
       found maximum in a and b. */
    max = (c > d? (c > max? c : max) : (d > max? d : max));
return max;
}

int main( int argc, char** argv )
{

 if(argc!=4){
        cout<<"Usage: ../seg input_image initialization_file output_mask"<<endl;
        return -1;
    }
    Mat in_image;
    in_image = imread(argv[1]/*, CV_LOAD_IMAGE_COLOR*/);
    Mat src, src_gray,grad;
    int scale = 1;
    int delta = 0;
    int depth = CV_16S;

    if(!in_image.data)
    {
        cout<<"Could not load input image!!!"<<endl;
        return -1;
    }

    if(in_image.channels()!=3){
        cout<<"Image does not have 3 channels!!! "<<in_image.depth()<<endl;
        return -1;
    }
    
    // the output image
    Mat out_image = in_image.clone();
    
    ifstream f(argv[2]);
    if(!f){
        cout<<"Could not load initial mask file!!!"<<endl;
        return -1;
    }
    
    int width = in_image.cols;
    int height = in_image.rows;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    cvtColor( in_image, src_gray, CV_BGR2GRAY );
    Sobel( src_gray, grad_x, depth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, depth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    int maximumWeight = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int temporaryMaximum = grad.at < uchar > (i, j);
			if (maximumWeight < temporaryMaximum) {
			maximumWeight = temporaryMaximum;
        }

    }
}

int  pixels = height * width;
	 vector<Vertex> graph((height*width) + 2, Vertex());
	for (int i = 0; i < pixels + 2; ++i) {
		graph.at(i).vertex = (i);
	}
	int source = pixels;
	int sink = pixels+1;

int maxw=0;
Mat M(height,width, CV_8SC1, Scalar(-1));

for(int i=0;i<pixels;++i){
    	int xpixel = i/width;
    	int ypixel = i%width;
	double weight = 0;
    	if(i-1>=0 && ypixel-1>=0){//edge to left
    		double edge_weight = abs(grad.at<uchar>(xpixel,ypixel) - grad.at<uchar>(xpixel,ypixel-1));
    		if(edge_weight<0.5){
 			weight = edge_weight + 100;
    			graph[i].edgeInsertion(Edge(i,i-1,INT16_MAX));
    		}else{
			 weight = edge_weight - 100;
    			graph[i].edgeInsertion(Edge(i,i-1,1));
    		}
    	}
    	if(i+1<pixels && ypixel+1<width){//edge to right
    		double edge_weight = abs(grad.at<uchar>(xpixel,ypixel) - grad.at<uchar>(xpixel,ypixel+1));
    		if(edge_weight<0.5){
 			weight = edge_weight + 100;
    			graph[i].edgeInsertion(Edge(i,i+1,INT16_MAX));
    		}else{
			weight = edge_weight - 100;
    			graph[i].edgeInsertion(Edge(i,i+1,1));
    		}
    	}
    	if(i-width>=0 && xpixel-1>=0){//edge to top
    		double edge_weight = abs(grad.at<uchar>(xpixel,ypixel) - grad.at<uchar>(xpixel-1,ypixel));
    		if(edge_weight<0.5){
			 weight = edge_weight + 100;
    			graph[i].edgeInsertion(Edge(i,i-width,INT16_MAX));
    		}else{
			weight = edge_weight - 100;
    			graph[i].edgeInsertion(Edge(i,i-width,1));
    		}
    	}
    	if(i+width<pixels && xpixel+1<height){//edge to bottom
    		double edge_weight = abs(grad.at<uchar>(xpixel,ypixel) - grad.at<uchar>(xpixel+1,ypixel));
    		if(edge_weight<0.5){
			 weight = edge_weight + 100;
    			graph[i].edgeInsertion(Edge(i,i+width,INT16_MAX));
    		}else{
			weight = edge_weight - 100;
    			graph[i].edgeInsertion(Edge(i,i+width,1));
    		}
    	}
    }
    int n;
    f>>n;
    
    // get the initil pixelsMat src, src_gray;
    for(int i=0;i<n;++i){
        int x, y, t;
        f>>x>>y>>t;
        
        if(x<0 || x>=width || y<0 || y>=height){
	cout<<"x"<<x<<"y"<<y<<endl;
            cout<<"I valid pixel mask!"<<endl;
            return -1;
        }
        
	int indexValue = y*width + x%width;
        
        Vec3b pixel;
        pixel[0] = 0;
        pixel[1] = 0;
        pixel[2] = 0;
        
        if(t==1){
            pixel[2] = 255;
	    graph[pixels].edgeInsertion(Edge(pixels,indexValue,INT16_MAX));
	    graph[indexValue].edgeInsertion(Edge(indexValue,pixels,INT16_MAX));
        } else {
            pixel[0] = 255;
	    graph[indexValue].edgeInsertion(Edge(indexValue,pixels+1,INT16_MAX));
	    graph[pixels+1].edgeInsertion(Edge(pixels+1,indexValue,INT16_MAX));
        }
        
        out_image.at<Vec3b>(y, x) = pixel;
    }
	 out_image = Mat::zeros(out_image.rows, out_image.cols, CV_8UC3);
	 mincutmaxflow(graph, source, sink,pixels,out_image,width);
     imwrite( argv[3], out_image);
    
    namedWindow( "Original image", WINDOW_AUTOSIZE );
    namedWindow( "Show Marked Pixels", WINDOW_AUTOSIZE );
    imshow( "Original image", in_image );
    imshow( "Show Marked Pixels", out_image );
    waitKey(0);
    return 0;
}



