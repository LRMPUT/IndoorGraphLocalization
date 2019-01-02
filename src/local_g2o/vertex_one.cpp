#include "local_g2o/vertex_one.h"

#include <typeinfo>
#include "g2o/stuff/macros.h"

namespace g2o {

    VertexOne::VertexOne() :
            BaseVertex<1, Vector1>()
    {
        _estimate.setZero();
    }

    bool VertexOne::read(std::istream& is)
    {
        is >> _estimate[0];
        return true;
    }

    bool VertexOne::write(std::ostream& os) const
    {
        os << estimate()(0);
        return os.good();
    }

} // end namespace