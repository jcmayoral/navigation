#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg> MySyncPolicy2;

typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg> MySyncPolicy3;

typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg> MySyncPolicy4;

typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg,
                                                        fusion_msgs::sensorFusionMsg> MySyncPolicy5;
