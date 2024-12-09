#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
using namespace std;
// compute ideal ratation and translation matrix
Eigen::Matrix4f createIdealTransformationMatrix(float tx, float ty, float tz, float rx, float ry, float rz)
{
    ////rotating and translating point cloud
    ////m and m1 are both the unit matrixes
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    // tx,ty,tz are translation values of x ,y and z axis
    m1(0, 3) = tx;
    m1(1, 3) = ty;
    m1(2, 3) = tz;
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the translation
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the x axis
    m1 = Eigen::Matrix4f::Identity();
    rx = rx / 180.0 * M_PI;
    m1(1, 1) = cos(rx);
    m1(1, 2) = -sin(rx);
    m1(2, 1) = sin(rx);
    m1(2, 2) = cos(rx);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along x axis
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the y axis
    m1 = Eigen::Matrix4f::Identity();
    ry = ry / 180.0 * M_PI;
    m1(0, 0) = cos(ry);
    m1(0, 2) = sin(ry);
    m1(2, 0) = -sin(ry);
    m1(2, 2) = cos(ry);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along y axis
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the z axis
    m1 = Eigen::Matrix4f::Identity();
    rz = rz / 180.0 * M_PI;
    m1(0, 0) = cos(rz);
    m1(0, 1) = -sin(rz);
    m1(1, 0) = sin(rz);
    m1(1, 1) = cos(rz);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along y axis
    // cout << "m" << endl << m << endl;
    return m;
}
// compute the error of rotation matrix and translation matrix
void computeMatrixError(float rotation, float translation, float &rotationError, float &translationError, Eigen::Matrix4f &realRtMatrix)
{
    Eigen::Matrix4f idealMatrix = Eigen::Matrix4f::Identity();
    idealMatrix = createIdealTransformationMatrix(translation, translation, translation, rotation, rotation, rotation);
    // cout << "idealMatrix:"<<endl << idealMatrix << endl;
    Eigen::Matrix3f idealRotationMatrix = Eigen::Matrix3f::Identity();
    Eigen::Vector3f iealTranslationVector = Eigen::Vector3f(0, 0, 0);
    // getting ideal rotation matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            idealRotationMatrix(i, j) = idealMatrix(i, j);
        }
    }
    // getting ideal translation matrix
    for (int i = 0; i < 3; i++)
    {
        iealTranslationVector(i) = idealMatrix(i, 3);
    }
    cout << "ideal Rotation matrix:" << endl
         << idealRotationMatrix << endl;
    cout << "ideal Translatin vector:" << endl
         << iealTranslationVector << endl;
    cout << "real Matrix:" << endl
         << realRtMatrix << endl;
    Eigen::Matrix3f realRotationMatrix = Eigen::Matrix3f::Identity();
    Eigen::Vector3f realTranslationVector = Eigen::Vector3f(0, 0, 0);
    // getting ideal rotation matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            realRotationMatrix(i, j) = realRtMatrix(i, j);
        }
    }
    // getting real translation matrix
    for (int i = 0; i < 3; i++)
    {
        realTranslationVector(i) = realRtMatrix(i, 3);
    }
    cout << "real Rotation matrix:" << endl
         << realRotationMatrix << endl;
    cout << "real Translatin vector:" << endl
         << realTranslationVector << endl;
    // compute rotation error
    Eigen::Matrix3f m = idealRotationMatrix * realRotationMatrix
                                                  .inverse();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(m);
    float tr = es.eigenvalues()(0) + es.eigenvalues()(1) + es.eigenvalues()(2);
    if (tr > 3)
        tr = 3;
    if (tr < -3)
        tr = -3;
    rotationError = acos((tr - 1) / 2) * 180.0 / M_PI;
    // compute translation error
    translationError = (realTranslationVector - iealTranslationVector).norm();
}