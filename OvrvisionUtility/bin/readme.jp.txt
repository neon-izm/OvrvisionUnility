�EOvrvision Stereo Chessboard Calibration Toolkit v0.2

Ovrvition�̍��E�J�����̓����p�����[�^�A�c�ݍs��A���ꂼ��̃J�����Ԃ̊֌W�s����v�Z���A�o�͂��郆�[�e�B���e�B�ł��B

�E�g����
0.chess4x7_30mm.pdf���v�����^��A4�Ɋg��k�������ň���i�����`�̈�ӂ�30mm�ɂȂ��Ă��邱�Ƃ��m�F���Ă��������j
1.Ovrvision���h������Ԃ�OvrvisionUtility.exe���N���B
2.��������`�F�X�{�[�h�����E�̃J�����������猩���悤���ӂ��āA�L�[�{�[�hc�������B������`�F�X�{�[�h�̌�����ς��Ȃ��畡����J��Ԃ�(10����x���ڈ�)
3.�L�[�{�[�hp�������ƁA�e��p�����[�^���t�@�C���ɏ����o�����B
4.��ʂ̃E�B���h�E���m�F���A�L�p�c�݂���������Ă��邱�Ƃ��m�F���A�L�[�{�[�hq�ŏI���B

�E�o�̓t�@�C��
intrinsics.yml�c�J���������p�����[�^�A�c�݌W��
stereoExtrinsics�c���E�J�����ԃp�����[�^
�E�⑫
��ʂ��Â��ꍇ�A�������Ă��������B
w�L�[�c���x+1
s�L�[�c���x-1
d�L�[�c�R���g���X�g+1
a�L�[�c�R���g���X�g-1

�E�L�����u���[�V�����̃R�c
�Ȃ�ׂ��J�����ɑ΂��ă`�F�X�{�[�h���傫���f��悤�ɂ���B
�l�X�Ȋp�x����B�e�ic�L�[�j���s���B���΂ߑO�A�E�΂ߑO�A���ʁA��΂ߑO���B
�B�e������10���ȏ�20���ȉ����炢���ڈ��B

�E�ݒ�t�@�C���̓ǂݍ��ݕ��̗�(OpenCV)
cv::FileStorage fs;
fs.open("intrinsics.yml", cv::FileStorage::READ);
 cv::Mat hoge; 
 fs["LeftCameraInstric"] >> M;
 