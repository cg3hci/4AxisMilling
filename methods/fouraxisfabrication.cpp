/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "fouraxisfabrication.h"


namespace FourAxisFabrication {


/* ----- FOUR AXIS FABRICATION ----- */

/**
 * @brief Compute the entire algorithm for four axis fabrication.
 *
 * @param[out] originalMesh Original mesh
 * @param[out] smoothedMesh Smoothed mesh
 * @param[in] nDirections Number of directions to check
 * @param[in] deterministic Deterministic approach (if false it is randomized)
 * @param[in] nDirections Number of directions
 * @param[in] fixExtremeAssociation Set if faces in the extremes must be already and unconditionally
 * assigned to the x-axis directions
 * @param[in] setCoverage Flag to resolve set coverage problem or not
 * @param[in] compactness Compactness
 * @param[in] limitAngle Limit angle
 * @param[in] frequenciesIterations Number of iterations of the algorithm for restoring frequencies
 * @param[in] occlusionsCheck Occlusion check in restoring frequncies
 * @param[in] recheckVisibility Update association after frequencies have
 * been restored (for debugging)
 * @param data Four axis fabrication data
 * @param[in] checkMode Visibility check mode. Default is projection mode.
 */
void computeEntireAlgorithm(
        cg3::EigenMesh& originalMesh,
        cg3::EigenMesh& smoothedMesh,
        const unsigned int nOrientations,
        const bool deterministic,
        const unsigned int nDirections,
        const bool fixExtremeAssociation,
        const bool setCoverage,
        const double compactness,
        const double limitAngle,
        const unsigned int frequenciesIterations,
        const bool occlusionsCheck,
        const bool recheckVisibility,
        Data& data,
        CheckMode checkMode)
{

    //Get optimal mesh orientation
    FourAxisFabrication::rotateToOptimalOrientation(
                originalMesh,
                smoothedMesh,
                nOrientations,
                deterministic);


    //Get extremes on x-axis to be cut
    FourAxisFabrication::selectExtremesOnXAxis(smoothedMesh, data);


    //Visibility check
    FourAxisFabrication::getVisibility(
                smoothedMesh,
                nDirections,
                fixExtremeAssociation,
                data,
                checkMode);



    //Get the target directions
    FourAxisFabrication::getTargetDirections(
                setCoverage,
                data);

    //Get optimized association
    FourAxisFabrication::getOptimizedAssociation(
                smoothedMesh,
                compactness,
                limitAngle,
                data);


    //Restore frequencies
    FourAxisFabrication::restoreFrequencies(
                frequenciesIterations,
                occlusionsCheck,
                originalMesh,
                smoothedMesh,
                data);

    //Get association after frequencies are restored
    FourAxisFabrication::calculateRestoredVisibility(
                recheckVisibility,
                data,
                checkMode);

    //Cut components
    FourAxisFabrication::cutComponents(
                data);


    //Cut components
    FourAxisFabrication::extractSurfaces(
                data);

}


}
