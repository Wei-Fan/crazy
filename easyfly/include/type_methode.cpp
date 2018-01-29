#include "type_methode.h"

/*void add_A2B(const Vector3f* a, Vector3f*b)
{
	for (int i=0; i<3; i++){
		(*b)(i)+=(*a)(i);
	}
}*/
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float deriv_f(const	float f_now, const float f_past , const float dt)
{
	return (f_now - f_past)/dt;
}
void vec3f_normalize(Vector3f* v)
{
	if((*v)(0)==0&&(*v)(1)==0&&(*v)(2)==0)
	{}
	else{
	float inv_norm;
	inv_norm = sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
	(*v)(0) /= inv_norm;
	(*v)(1) /= inv_norm;
	(*v)(2) /= inv_norm;
	}
}
void vec3f_norm(const Vector3f* a, float* anwser)
{
	*anwser = sqrtf((*a)(0)*(*a)(0) + (*a)(1)*(*a)(1) + (*a)(2)*(*a)(2));
}
void vec3f_cross(const Vector3f* a, const Vector3f* b, Vector3f* d)
{
/*
a X b = | i		j		k	|
		| ax	ay		az	|
		| bx	by		bz	|
	where |.| is the determinant
*/	
	(*d)(0) = (*a)(1) * (*b)(2) - (*a)(2) * (*b)(1);
	(*d)(1) = (*a)(2) * (*b)(0) - (*a)(0) * (*b)(2);
	(*d)(2) = (*a)(0) * (*b)(1) - (*a)(1) * (*b)(0);
}
float vec3f_dot(const Vector3f* a, const Vector3f* b)
{
	return ((*a)(0) * (*b)(0) + (*a)(1) * (*b)(1) + (*a)(2) * (*b)(2));
}
void vec3f_passnorm(const Vector3f* v, Vector3f* vec_des)
{
	float inv_norm;
	inv_norm = sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
	(*vec_des)(0) = (*v)(0) / inv_norm;
	(*vec_des)(1) = (*v)(1) / inv_norm;
	(*vec_des)(2) = (*v)(2) / inv_norm;
}

void vec3f_integration(Vector3f* Integrated, Vector3f* Origin, float dt)
	{
		(*Integrated)(0) += (*Origin)(0)*dt;
		(*Integrated)(1) += (*Origin)(1)*dt;
		(*Integrated)(2) += (*Origin)(2)*dt;
	}

void vec3f_derivative(Vector3f* Deriv, Vector3f* Origin, Vector3f* l_Origin, float dt)
{
		(*Deriv)(0) = ((*Origin)(0) - (*l_Origin)(0))/dt;
		(*Deriv)(1) = ((*Origin)(1) - (*l_Origin)(1))/dt;
		(*Deriv)(2) = ((*Origin)(2) - (*l_Origin)(2))/dt;

}

float vec3f_length(const Vector3f* v)
{
	return sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
}
void body2earth(const Matrix3f* R, const Vector3f* body, Vector3f* earth, short dimension)
{	
	if(dimension == 2){
		float yaw = -atan2((*R)(0,1), (*R)(1,1));
		(*earth)(0) = (*body)(0)*cos(yaw) + (*body)(1)*sin(-yaw);
		(*earth)(1) = (*body)(0)*sin(yaw) + (*body)(1)*cos(yaw); 
	}
	else if(dimension == 3){
		(*earth)(0) = ((*body)(0)*(*R)(0,0) + (*body)(1)*(*R)(0,1) + (*body)(2)*(*R)(0,2));
		(*earth)(1) = ((*body)(0)*(*R)(1,0) + (*body)(1)*(*R)(1,1) + (*body)(2)*(*R)(1,2));
		(*earth)(2) = ((*body)(0)*(*R)(2,0) + (*body)(1)*(*R)(2,1) + (*body)(2)*(*R)(2,2));
	}
}
void earth2body(const Matrix3f* R, const Vector3f* earth, Vector3f* body, short dimension)//body=inv(R)*earth
{	
	if(dimension == 2){
		float yaw = -atan2((*R)(0,1), (*R)(1,1));
		(*body)(0) = (*earth)(0)*cos(yaw) + (*earth)(1)*sin(yaw);
		(*body)(1) = (*earth)(0)*sin(-yaw) + (*earth)(1)*cos(yaw); 
	}
	else if(dimension == 3){
		(*body)(0) = ((*earth)(0)*(*R)(0,0) + (*earth)(1)*(*R)(1,0) + (*earth)(2)*(*R)(2,0));
		(*body)(1) = ((*earth)(0)*(*R)(0,1) + (*earth)(1)*(*R)(1,1) + (*earth)(2)*(*R)(2,1));
		(*body)(2) = ((*earth)(0)*(*R)(0,2) + (*earth)(1)*(*R)(1,2) + (*earth)(2)*(*R)(2,2));
	}
}
void rotation2euler(const Matrix3f* R, Vector3f* Euler)
{
	(*Euler)(1) = -asin((*R)(2,0));
	(*Euler)(0) = atan2((*R)(2,1), (*R)(2,2));
	(*Euler)(2) = -atan2((*R)(0,1), (*R)(1,1));
}
void euler2rotation(const Vector3f* Euler, Matrix3f* R)
{
	float cp = cos((*Euler)(1));
	float sp = sin((*Euler)(1));
	float sr = sin((*Euler)(0));
	float cr = cos((*Euler)(0));
	float sy = sin((*Euler)(2));
	float cy = cos((*Euler)(2));
	(*R)(0,0) = cp * cy;
	(*R)(0,1) = -((sr * sp * cy) - (cr * sy));
	(*R)(0,2) = ((cr * sp * cy) + (sr * sy));
	(*R)(1,0) = cp * sy;
	(*R)(1,1) = ((sr * sp * sy) + (cr * cy));
	(*R)(1,2) = ((cr * sp * sy) - (sr * cy));
	(*R)(2,0) = -sp;
	(*R)(2,1) = sr * cp;
	(*R)(2,2) = cr * cp;	
}
void quaternion2rotation(const Vector4f* Q, Matrix3f* R)
{
	float q0,q1,q2,q3;
	q0 = (*Q)(0);
	q1 = (*Q)(1);
	q2 = (*Q)(2);
	q3 = (*Q)(3);
	(*R)(0,0)=1.0f - ((q2 * q2 + q3 * q3) * 2.0f);
	(*R)(0,1)=(q1 * q2 - q0 * q3) * 2.0f;
	(*R)(0,2)=(q1 * q3 + q0 * q2) * 2.0f;
	(*R)(1,0)=(q1 * q2 + q0 * q3) * 2.0f;
	(*R)(1,1)=1.0f - ((q1 * q1 + q3 * q3) * 2.0f);
	(*R)(1,2)=(q2 * q3 - q0 * q1) * 2.0f;
	(*R)(2,0)=(q1 * q3 - q0 * q2) * 2.0f;
	(*R)(2,1)=(q2 * q3 + q0 * q1) * 2.0f;
	(*R)(2,2)=1.0f - ((q1 * q1 + q2 * q2) * 2.0f);  
}

void euler2quaternion(const Vector3f* Euler, Vector4f* Q)
{
	float hr = (*Euler)(0) * 0.5f;
	float hp = (*Euler)(1) * 0.5f;
	float hy = (*Euler)(2) * 0.5f;
	float q0,q1,q2,q3;
	float shr = sin(hr);
	float chr = cos(hr);
	float shp = sin(hp);
	float chp = cos(hp);
	float shy = sin(hy);
	float chy = cos(hy);
	/*
	q0 = (cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy));  
	q1 = (sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy));    
	q2 = (cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy)); 
	q3 = (cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy));
	*/
	q0 = chr*chp*chy + shr*shp*shy;  
	q1 = shr*chp*chy - chr*shp*shy;    
	q2 = chr*shp*chy + shr*chp*shy; 
	q3 = chr*chp*shy - shr*shp*chy;
	(*Q)(0) = q0;
	(*Q)(1) = q1;
	(*Q)(2) = q2;
	(*Q)(3) = q3;
}
void rotation2quaternion(const Matrix3f* R, Vector4f* Q)
{
	float q[4];
	float tr = (*R)(0,0) + (*R)(1,1) + (*R)(2,2);
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = ((*R)(2,1) - (*R)(1,2)) * s;
		q[2] = ((*R)(0,2) - (*R)(2,0)) * s;
		q[3] = ((*R)(1,0) - (*R)(0,1)) * s;
	} 
	else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int dcm_i = 0;
		for (int i = 1; i < 3; i++) {
			if ((*R)(i,i) > (*R)(dcm_i,dcm_i)) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf(((*R)(dcm_i,dcm_i) - (*R)(dcm_i,dcm_j) - (*R)(dcm_k,dcm_k)) + 1.0f);
		q[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q[dcm_j + 1] = ((*R)(dcm_i,dcm_j) + (*R)(dcm_i,dcm_i)) * s;
		q[dcm_k + 1] = ((*R)(dcm_k,dcm_i) + (*R)(dcm_i,dcm_k)) * s;
		q[0] = ((*R)(dcm_k,dcm_j) - (*R)(dcm_i,dcm_k)) * s;
	}
	for(int i = 0; i < 4; i++)
		(*Q)(i) = q[i];
}

/*void quaternion_derivative(const Vector4f* Q, Vector4f* derQ, const Vector3f* w)
{
	int i;
	float dataQ[] = {
		(*Q)(0), -(*Q)(1), -(*Q)(2), -(*Q)(3),
		(*Q)(1),  (*Q)(0), -(*Q)(3),  (*Q)(2),
		(*Q)(2),  (*Q)(3),  (*Q)(0), -(*Q)(1),
		(*Q)(3), -(*Q)(2),  (*Q)(1),  (*Q)(0)
	};
	float V[4] = {0,(*w)(0),(*w)(1),(*w)(2)};
	float Q_V[4];
	float result[4];
	arm_matrix_instance_f32 matQ = {4,4,(float *)dataQ};
	arm_matrix_instance_f32 matV = {4,1,(float *)V};
	arm_matrix_instance_f32 matQ_V = {4,1,(float *)Q_V};
	arm_matrix_instance_f32 matRes = {4,1,(float *)result};
	arm_mat_mult_f32(&matQ,&matV,&matQ_V);
	arm_mat_scale_f32(&matQ_V,0.5f,&matRes);
	for(i=0;i<4;i++){
		derQ->q[i] = result[i];
	}
}*/
void quaternion_normalize(Vector4f* Q)
{
	float q0,q1,q2,q3;
	float inv_norm;
	q0 = (*Q)(0);
	q1 = (*Q)(1);
	q2 = (*Q)(2);
	q3 = (*Q)(3);
	float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;
	(*Q)(0) = q0;
	(*Q)(1) = q1;
	(*Q)(2) = q2;
	(*Q)(3) = q3;
}


/*void resetM_Qdot2Q(const Vector3f* gyro, Matrix4f* M_Qdot2Q)
{
	M_Qdot2Q->setZero();
	M_Qdot2Q->block(0,1,3,1)
}*/

float data_2_angle(float x, float y, float z)	 //in rad
{
	float res;
	res = atan2(x,sqrtf(y*y+z*z));
	return res;
}
float degToRad(float deg) {
    return deg / 180.0f * M_PI;
}

float radToDeg(float rad) {
    return rad * 180.0f / M_PI;
}
void writeData_bin(const char* fname, Vector3f* vec)//, int num_Data)
{	
	//rename
	/*char fname[250];
	sprintf(fname,"%s%s",m_resFnameRoot,fname_input);*/
	//create file
	FILE* file_ptr;
	file_ptr = fopen(fname,"ab");
	if( file_ptr == NULL){
		file_ptr = fopen(fname,"wb");
	}
	char inputf[50];
	sprintf(inputf,"%f  %f  %f\n",(*vec)(0),(*vec)(1),(*vec)(2));
	fputs (inputf,file_ptr);
	fclose (file_ptr);
}
void writeData_binf(const char* fname, float number)
{	
	//rename
	/*char fname[250];
	sprintf(fname,"%s%s",m_resFnameRoot,fname_input);*/
	//create file
	FILE* file_ptr;
	file_ptr = fopen(fname,"ab");
	if( file_ptr == NULL){
		file_ptr = fopen(fname,"wb");
	}
	char inputf[50];
	sprintf(inputf,"%f\n",number);
	fputs (inputf,file_ptr);
	fclose (file_ptr);
}

void vec3f_angle(Vector3f* a, Vector3f* b, float* angleInRad)
{
	float norm_a, norm_b;
	vec3f_norm(a,&norm_a);
	vec3f_norm(b,&norm_b);
	*angleInRad = acos(vec3f_dot(a,b)/(norm_a*norm_b));
}
bool IsUAVForm(Vector3f* a, Vector3f*b)
{
	float length_Rate = vec3f_length(a)/vec3f_length(b);
	float angle;
	vec3f_angle(a, b, &angle);

	if((((1/1.41421356-0.2<length_Rate<1/1.41421356+0.2)||(1.41421356-0.2<length_Rate<1.41421356+0.2))&&(M_PI/4-0.1<angle<M_PI+0.10))||(0.8<length_Rate<1.2 && M_PI/2-0.4<angle<M_PI/2+0.4))
	{
		return true;
	}
	else
	{
		return false;
	}
	
}
Vector3f CreateVectorFromTwoPts(Vector3f* a, Vector3f* b)
{
	Vector3f result;
	result(0) = (*b)(0) - (*a)(0);
	result(1) = (*b)(1) - (*a)(1);
	result(2) = (*b)(2) - (*a)(2);
	return result;
}
void number_times_vec3f(float* a, Vector3f* v)
{
	(*v)(0) = (*a) * (*v)(0);
	(*v)(1) = (*a) * (*v)(1); 
	(*v)(2) = (*a) * (*v)(2);
}