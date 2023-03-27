#include "so3.h"

int main()
{
	std::cout<<" --------------------- SO3 TEST --------------------"<<std::endl;

	slam::SO3<double> mat_A;

	Eigen::Vector3d rotation_vec(  -0.43408, -1.12137, -1.15755 );
	std::cout<<"rotation vec = "<<rotation_vec.transpose()<<std::endl;
	

	mat_A = slam::SO3<double>::exp( rotation_vec );

	std::cout<<"matrix : "<<std::endl<<mat_A.getRotationMatrix()<<std::endl;

	Eigen::Vector3d rotation_vec_src = slam::SO3<double>::log( mat_A );

	std::cout<<"rotation vec = "<<rotation_vec_src.transpose()<<std::endl;

	std::cout<<" ---------------------- SE3 TEST -----------------"<<std::endl;
	
	Eigen::Vector3d translation_vec( 1, 1, 1 );
	slam::SE3<double> mat_B( rotation_vec, translation_vec );

	Eigen::Matrix<double, 6, 1> se3 = slam::SE3<double>::log( mat_B );
	std::cout<<"se3 = "<<se3.transpose()<<std::endl;

	slam::SE3<double> mat_BB = slam::SE3<double>::exp( se3 );
	std::cout<<"SE3 = "<<std::endl<<mat_BB.getTransformMatrix()<<std::endl;
	
	return 0;
}
