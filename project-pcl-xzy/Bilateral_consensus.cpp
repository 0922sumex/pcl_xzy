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
	//���ѡr��ƽ��ŵ�����PLFH_randomselect��
	vector<pcl::PLFH_gather> PLFH_randomselect;
	PLFH_randomselect.clear();

	int max = plfh_set_S.size();
	int x = -1;
	for (int i = 0; i < r; i++) {
		 x = rand() % (max + 1);
		 if (is_element_in_vector(PLFH_randomselect, plfh_set_S[x]) != true) {
			 //Ԫ�ز���������
			 PLFH_randomselect.push_back(plfh_set_S[x]);
		 }
		 else {
			 r++;
		 }
	}
	return PLFH_randomselect;
}

double CalcuNorms(pcl::PLFH_gather plfh_set_A, pcl::PLFH_gather plfh_set_B) {
	//��������AB��Ķ�����
	VectorXf BilateralCon_AB(18);
	for (int i = 0; i < plfh_set_A.features_histogram.size(); i++) {
		BilateralCon_AB(i) = plfh_set_A.features_histogram[i] - plfh_set_B.features_histogram[i];
	}
	return BilateralCon_AB.norm();
}
int CalcuIndex(pcl::PLFH_gather plfh_set_A, vector<pcl::PLFH_gather> plfh_set_B) {
	//plfh�������˫�߹�ʶ
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
	PLFH_randomselect=RandomSelect(plfh_set_S, R_PLANE_NUMBER);//ѡ4��ƽ��
	
	Vector2d BilateralCon_AB(2);
	vector<Vector2d> BilCon_index_set;//�洢�õ���˫�߹�ʶ������
	BilCon_index_set.clear();

	for (int i = 0; i < R_PLANE_NUMBER; i++) {
		BilateralCon_AB(0)=CalcuIndex(PLFH_randomselect[i],plfh_set_T);
		BilateralCon_AB(1)= CalcuIndex(plfh_set_T[BilateralCon_AB(0)], plfh_set_S);
		BilCon_index_set.push_back(BilateralCon_AB);

		//cout << "��"<<i<<"�Ծ�����С��PLFH��" << endl;
		//cout << "Դƽ������ţ�" << BilCon_index_set[i](1) << "   Ŀ��ƽ������ţ�" << BilCon_index_set[i](0) << endl;
	}
}