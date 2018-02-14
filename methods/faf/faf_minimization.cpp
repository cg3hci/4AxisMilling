/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_minimization.h"

//Uncomment if you want information on console
//#define MINIMIZATION_VERBOSE

#ifdef GUROBI_DEFINED
#include <gurobi_c++.h>
#endif

namespace FourAxisFabrication {

/* Minimization of the target fabrication directions */

/**
 * @brief Get the target fabrication directions.
 * We can keep all the directions or we get the minimum number of directions
 * which covers all the faces.
 * We solve a set coverage problem with gurobi.
 * @param[in] setCoverage Flag to compute the set coverage, otherwise we keep each direction
 * @param[out] data Four axis fabrication data
 */
void getTargetDirections(
        const bool setCoverage,
        Data& data)
{
    const cg3::Array2D<int>& visibility = data.visibility;
    std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;

    //Clear current data (if any)
    targetDirections.clear();

    int nOrientations = visibility.getSizeX();

    //We keep each direction if it is chosen or if there are non-visible triangles
    if (!setCoverage || nonVisibleFaces.size() > 0) {
        for (unsigned int i = 0; i < (unsigned int)nOrientations; i++)
            targetDirections.push_back(i);
    }
    else {

#ifdef GUROBI_DEFINED

        int nFaces = visibility.getSizeY();

        //If gurobi is defined, we solve the set coverage problem
        try {
            //Initialize gurobi
            GRBEnv env = GRBEnv();
            GRBModel model = GRBModel(env);
            GRBVar *orientation = model.addVars(nOrientations, GRB_BINARY);

            //Create variables for orientations and triangles
            for (int i = 0; i < nOrientations; i++) {
                std::ostringstream vname;
                vname << "o" << i;
                orientation[i].set(GRB_StringAttr_VarName, vname.str());
            }

            model.update();

            //We add constraints
            for(int i = 0; i < nFaces; i++) {
                GRBLinExpr sum = 0;
                for(int j = 0 ; j < nOrientations ; j++){
                    sum += orientation[j] * visibility(j,i);
                }
                model.addConstr(sum >= 1);
            }

            //We set the objective function
            GRBLinExpr expr = 0;
            for (int j = 0; j < nOrientations; j++) {
                expr += orientation[j];
            }
            model.setObjective(expr, GRB_MINIMIZE);

            model.optimize();

            for (int i = 0; i < nOrientations; i++) {
#ifdef MINIMIZATION_VERBOSE
                std::cout <<
                             orientation[i].get(GRB_StringAttr_VarName) << " " <<
                             orientation[i].get(GRB_DoubleAttr_X) << std::endl;
#endif
                if(orientation[i].get(GRB_DoubleAttr_X) == 1)
                    targetDirections.push_back(i);
            }
        }
        //Error handling
        catch (GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;

            targetDirections.clear();
        }
        catch (...) {
            std::cout << "Exception during optimization" << std::endl;

            targetDirections.clear();
        }

#else
        //If gurobi is not available, we keep each direction
        for (unsigned int i = 0; i < (unsigned int)nOrientations; i++)
            targetDirections.push_back(i);
#endif
    }
}


}
