#include "local_g2o/edge_wall.h"

namespace g2o{

    EdgeWall::EdgeWall() : BaseMultiEdge<-1,VectorX>(){
        resize(0);
    }

    void EdgeWall::computeError(){
        VertexPointXY * W1 = static_cast<VertexPointXY *> (_vertices[0]);
        VertexPointXY * W2 = static_cast<VertexPointXY *> (_vertices[1]);
        Point w1 = {W1->estimate()[0], W1->estimate()[1]};
        Point w2 = {W2->estimate()[0], W2->estimate()[1]};

        //printf("EDGE_WALL: wall : (%f %f) (%f %f)\n", w1.x, w1.y, w2.x, w2.y);

        VertexSE2 * P1 = static_cast<VertexSE2 *> (_vertices[2]);
        VertexSE2 * P2 = static_cast<VertexSE2 *> (_vertices[3]);
        Point p1 = {P1->estimate()[0], P1->estimate()[1]};
        Point p2 = {P2->estimate()[0], P2->estimate()[1]};

      //  printf("EDGE_WALL: poses : (%f %f) (%f %f)\n", p1.x, p1.y, p2.x, p2.y);

        if ( doIntersect(w1, w2, p1, p2) ) {

            double d1 = distanceToLine(w1, w2, p1);
            double d2 = distanceToLine(w1, w2, p2);
       //     printf("EDGE_WALL: doIntersect! : d1=%f, d2=%f, pen=%f\n", d1, d2, _measurement[0]);

            _error[0] = _measurement[0] * min(d1,d2);
        }
        else {
     //       printf("EDGE_WALL: wall inactive!\n");
            _error[0] = 0;
        }
     //   printf("EDGE_WALL: _error[0] : (%f)\n", _error[0]);



//        for(unsigned int i=0; i<_kDimension; i++){
//            VertexPointXY * xy = static_cast<VertexPointXY *> (_vertices[1+i]);
//
//            printf("EDGE_WKNN: vertices id = %d | %f %f\n", xy->id(), xy->estimate()[0], xy->estimate()[1]);
//
//            XY[0] += _measurement[i] * xy->estimate()[0];
//            XY[1] += _measurement[i] * xy->estimate()[1];
//        }
//
//        XY[0] = XY[0] / _sumOfMeasurements;
//        XY[1] = XY[1] / _sumOfMeasurements;
//
//        printf("EDGE_WKNN: XY = %f %f\n", XY[0], XY[1]);
//        printf("EDGE_WKNN: POSE ESTIMATE = %f %f\n", pose->estimate()[0], pose->estimate()[1]);
//
//        _error[0] = pose->estimate()[0] - XY[0];
//        _error[1] = pose->estimate()[1] - XY[1];
//        printf("EDGE_WKNN: Error = %f %f\n", _error[0], _error[1]);
    }

    // EdgeWall kDiemnsion W1 W2 P1 P2 kDim PEN inf_matrix
    bool EdgeWall::read(std::istream& is){
        is >> _kDimension;
        setSize(_kDimension);
        //printf("EDGE_WALL: _kDimension = %d\n", _kDimension);

        // read the measurements
        for(unsigned int i=0; i<_kDimension; i++){
            is >> _measurement[i];
        }
        //printf("EDGE_WALL: _measurement[0] = %f\n", _measurement[0]);

        // read the information matrix
        for(unsigned int i=0; i<1; i++){
            // fill the "upper triangle" part of the matrix
            for(unsigned int j=i; j<1; j++){
                is >> information()(i,j);
            }

            // fill the lower triangle part
            for(unsigned int j=0; j<i; j++){
                information()(i,j) = information()(j,i);
            }

        }

        return true;
    }

    bool EdgeWall::write(std::ostream& os) const{
        // write number of observed points
        os << _kDimension;

        // write measurements
        for(unsigned int i=0; i<_kDimension; i++){
            os << " " << _measurement[i];
        }

        // write information matrix
        for(unsigned int i=0; i<1; i++){
            for(unsigned int j=i; j<1; j++){
                os << " " << information()(i,j);
            }
        }

        return os.good();
    }


    //void EdgeWall::linearizeOplus(){
//
//        MatrixXD Ji;
//        Ji.resize(2, 2);
//        Ji.fill(0);
//        Ji(0, 0) = 1.0;
//        Ji(1, 1) = 1.0;
//
//        _jacobianOplus[0] = Ji;
//
//        for(unsigned int i=1; i<_vertices.size(); i++){
//            MatrixXD Jj;
//            Jj = -_measurement[i-1]/_sumOfMeasurements * Ji;
//            _jacobianOplus[i] = Jj;
//        }
  //  }


    // TODO
    void EdgeWall::initialEstimate(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
        printf("Edge_WKNN : initialEstimate\n");
        (void) toEstimate;

        printf("Edge_WKNN : initialEstimate = %d vertices = %d\n", fixed.size(), _vertices.size());

//        assert(initialEstimatePossible(fixed, toEstimate) && "Bad vertices specified");
//
//#ifdef _MSC_VER
//        std::vector<bool> estimate_this(_kDimension, true);
//#else
//        bool estimate_this[_kDimension];
//        for(unsigned int i=0; i<_kDimension; i++){
//            estimate_this[i] = true;
//        }
//#endif
//
//        for(std::set<HyperGraph::Vertex*>::iterator it=fixed.begin(); it!=fixed.end(); it++){
//            for(unsigned int i=0; i<_vertices.size(); i++){
//                VertexPointXY * vert = static_cast<VertexPointXY *>(_vertices[i]);
//                if(vert->id() == (*it)->id())
//                    _vertices[i] = *it;
//            }
//        }
    }


    double EdgeWall::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
        (void) toEstimate;

        for(std::set<HyperGraph::Vertex *>::iterator it=fixed.begin(); it!=fixed.end(); it++){
            if(_vertices[0]->id() == (*it)->id()){
                return 1.0;
            }
        }

        return -1.0;
    }

    // TODO: HOW?
    bool EdgeWall::setMeasurementFromState(){
//	  VertexPointXY * pose = static_cast<VertexPointXY *> (_vertices[0]);
//
//    for(unsigned int i=0; i<_kDimension; i++){
//      VertexPointXY * xy = static_cast<VertexPointXY *> (_vertices[1+i]);
//      Vector2D m = pose->estimate().inverse() * xy->estimate();
//
//      unsigned int index = 2*i;
//      _measurement[index] = m[0];
//      _measurement[index+1] = m[1];
//    }
        printf("Edge_Wall: setMeasurementFromState");
        for (unsigned int i = 0; i < _kDimension; i++) {
            _measurement[i] = 10;
        }

        return true;
    }

} // end namespace local_g2o