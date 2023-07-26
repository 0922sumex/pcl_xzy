#include "Bilateral_consensus.h"

bool is_element_in_vector(vector<pcl::PLFH_gather> v, pcl::PLFH_gather element) {
	vector<pcl::PLFH_gather>::iterator it;
	for (int i = 0; i < v.size(); i++) {
		if (v[i].features_histogram==element.features_histogram) {
			return true;
		}
	}
	return false;
}

vector<pcl::PLFH_gather> RandomSelect(vector<pcl::PLFH_gather> plfh_set_S,int r) {
	//随机选r个平面放到集合PLFH_randomselect里
	vector<pcl::PLFH_gather> PLFH_randomselect;
	PLFH_randomselect.clear();

	int max = plfh_set_S.size();
	int x = -1;
	for (int i = 0; i < r; i++) {
		 x = rand() % (max + 1);
		 if (is_element_in_vector(PLFH_randomselect, plfh_set_S[x]) != true) {
			 //元素不在容器里
			 PLFH_randomselect.push_back(plfh_set_S[x]);
		 }
		 else {
			 r++;
		 }
	}
	return PLFH_randomselect;
}

double CalcuNorms(pcl::PLFH_gather plfh_set_A, pcl::PLFH_gather plfh_set_B) {
	//计算向量AB间的二范数
	VectorXf BilateralCon_AB(18);
	for (int i = 0; i < plfh_set_A.features_histogram.size(); i++) {
		BilateralCon_AB(i) = plfh_set_A.features_histogram[i] - plfh_set_B.features_histogram[i];
	}
	return BilateralCon_AB.norm();
}
int CalcuIndex(pcl::PLFH_gather plfh_set_A, vector<pcl::PLFH_gather> plfh_set_B) {
	//plfh向量间的双边共识
	double temp_norm;
	double temp_compare = 10000.0;
	int index_T = -1;
	for (int i = 0; i < plfh_set_B.size(); i++) {
		temp_norm = CalcuNorms(plfh_set_A,plfh_set_B[i]);
		if (temp_norm < temp_compare) {
			temp_compare = temp_norm;
			index_T = i;
		}
	}
	return index_T;
}

void BilateralConsensus(vector<pcl::PLFH_gather> plfh_set_S, vector<pcl::PLFH_gather> plfh_set_T) {
	vector<pcl::PLFH_gather> PLFH_randomselect;
	PLFH_randomselect.clear();
	PLFH_randomselect=RandomSelect(plfh_set_S, R_PLANE_NUMBER);//选4个平面
	
	Vector2d BilateralCon_AB(2);
	vector<Vector2d> BilCon_index_set;//存储得到的双边共识的索引
	BilCon_index_set.clear();

	for (int i = 0; i < R_PLANE_NUMBER; i++) {
		BilateralCon_AB(0)=CalcuIndex(PLFH_randomselect[i],plfh_set_T);
		BilateralCon_AB(1)= CalcuIndex(plfh_set_T[BilateralCon_AB(0)], plfh_set_S);
		BilCon_index_set.push_back(BilateralCon_AB);

		//cout << "第"<<i<<"对距离最小的PLFH：" << endl;
		//cout << "源平面中序号：" << BilCon_index_set[i](1) << "   目标平面中序号：" << BilCon_index_set[i](0) << endl;
	}
}