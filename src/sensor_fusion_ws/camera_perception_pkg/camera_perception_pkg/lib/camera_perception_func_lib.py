import cv2
import numpy as np
import sys
import os
from typing import Tuple
import rclpy.logging as _rclpy_logging

# [곡선 인코스 진단용 임시 로그] get_lane_center()가 '두 선 보임' 분기로 새면
# tilt_comp가 아예 적용되지 않는다. 실주행에서 어느 분기가 도는지 확인하려고 넣었다.
# 확인 끝나면 이 카운터/로그 블록은 지운다.
_debug_call_counter = 0

message = """

 _____ _     _         _   ___                           _
|_   _| |__ (_)_ __ __| | |_ _|_ __ ___  _ __   __ _  __| |_
  | | | '_ \| | '__/ _` |  | || '_ ` _ \| '_ \ / _` |/ _` __|
  | | | | | | | | | (_| |  | || | | | | | |_) | (_| | (_| |_
  |_| |_| |_|_|_|  \__,_| |___|_| |_| |_| .__/ \__,_|\__,_(_)
                                         |_|
 _____                            _ _
| ____|_   ____ _ _ __   __ _  ___| (_) ___  _ __
|  _| \ \ / / _` | '_ \ / _` |/ _ \ | |/ _ \| '_ \\
| |___ \ V / (_| | | | | (_| |  __/ | | (_) | | | |
|_____| \_/ \__,_|_| |_|\__, |\___|_|_|\___/|_| |_|
                        |___/

.:::::::....::::::::::::::::.                          .  .       .. .      ..                           ----::::::::::::::::.... 
............:::::::::::::::::                            ..::.:.  ..-:.:..:.                            :--:::::::::::::...... ...
.........:::--------:::::::::                            . ....     .  . ...  .                         ::::::-::::::.............
:::::::::---------:--------::                       ..   .........:...:..     ...                       -----::.............. ... 
.......:::::::::-----:::::::-:                   ..... .. .:===-=++==:==:-..  ......                   -----:...........        ..
...........::::::::::::::::...                   .=:.:..:.=##***####**#*#*=:   ...-.                .-:=--:...........          ..
    ..........:......::.......                  :.-:.:.::-=##%%#*+########+-   ..:=::             .----:...........           ....
       ..........::............              :---.:-::.  .+##++**+*#######=   ...--.---:          .....    .               ...:...
      .........................              -----:-:::::+*#####++#**####+#+.::::=:----:.         .....               ............
   ..........................               ------.:-:=*#######*-===-+###%*###+--:.------                         .......         
                                            -----.-########%%*##+-===###%%#%######+.-----:                    .....               
                                  .        -----*###%%%%%##%%#-+#####*+%%%##%%%%####*-----.                                       
                                          ---=+##%#%%%%%%###%%+***####*###%#%%%%%%#%##++=-:.                                      
                    .     .             --+***##%%###%%%%%%#%%#=-**==#%%###%#%%%#######+**++-                                     
       ..... ......   ....             .++*+*####%%%%#%%%%%%%%%#*:::=#%%%%%%%%%#%%%%%%%%#+*++:                                    
                                        +++#%%%%%%%%%#%%%%%%%%%%%=:-#%%%%%%%%%%#%%%######*+++.                                    
                   ....                 ++*%%%%%%%###%%%%%%%%%%%%%*#%%%%%%%%%%%%%#%%%%%%%%#++                                     
             .........                  -*+-*%%%%%%%%%*%%%%%%%%%%%%%#%%%%%%#%#%#%%%%%%%%##=**                                     
          .............                 :++#####%%%%%-=#%%%%%%%%%%###%%%%%%#%**=*%%%%#####+=-                                     
      .............                     -+####+*#%%%...=#%%%%%%%%%#%%%%%%%%#%-...#%%###+##*+=                       ..            
           .                           =+**+++##**%::...*#%%%%%######%####%%#. .::%*####+++*++.              ......               
                                    :++++++++##+#+:::::::###################=.:::::+*+=###++++++:       .........         ..      
                           ..     -+++++++*##*::::::::-=-+%%%*::......:*%%%%---:.:::::.+####*+++++:     ...::...........          
                        ......   =+++++*####**=.:::. -+++.:+:----:::---=-=-.==+-: :::.-++######++++=   ....... .....  ........   .
                      .......   =+++########+*+... :::--:  ------:::----++: .=+---..:.=+++#######*++=  .           ...............
                     ..........:+*#########.=+.  --------:-------:::-----++=:+=-=+=-.  ** =########*+-          ....    ......... 
        ..           .........:+##########.  .  -:-:----:-------:.::.:----+*=-+*=-+++      -########*+-......                     
.........    ........:::......+#########=        ------:--------:....-----***+=+*=:=-   ..   *#######*+:                          
.::.........:::::.:::....:. :+########+    :    ---:---------------------+****++*=+++.   =    .*######*+-           .             
...:::....::...:++-.::::.  =+#######*.    ..    --:--=--==--------------+***++--++-++.    -     .*#####*+=    ....:++=.           
    .........:=++***=:.  :+*######*.      :     :+++++--+=-------------+*++++:=+++++-     .-      :#####*++.   .-+++=++=.        .
       .....-+*++*+#*--=+=+######=       ++.      ..     :-------------=+++=      .      .=+-      .*####*+++++==*+++++++:.    ...
       ...:--=+++++++-=+++*#####-       =+.              :--------------++++               =+:       *######*+=--+++++++-:::......
      .:---+++++++=--*##**#####-       =:                :--------------++++                .=.       +*####*##*-::++++++=----:...
   ..--++++++++++=#*###**++=*#.            .....         ---------------=+++:           ....           -*=+=+#***+#=++++++++++++-.
. ..-++++++++++++==**+*##**=+.       .. .........        ----------------+++=         .......           -=**+#****===++++++++++++-


"""
print(message)

print("")
print("Unita of Inchon National University")

print("------------------Authors------------------")
print("Wonjong Lee <leewon011002@gmail.com>")
print("Kyumin Jeong <jkmin0102@gmail.com>")
print("Seokbin Lee <leskbn011@naver.com>")
print("Hyunyoung Sung <imsunghy@gmail.com>")
print("JuHyeong Han <hanjuhyeong25@gmail.com>")
print("------------------------------------------")

def dominant_gradient(image, theta_limit):
	right_limit_radian = np.deg2rad(90+(90-theta_limit))
	left_limit_radian = np.deg2rad(90-(90-theta_limit))

	(height, width) = (image.shape[0], image.shape[1])

	if image.dtype != np.uint8:
		image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

	image_original = image.copy()

	try:
		lines = cv2.HoughLines(image, 1, np.pi/180, int(width*(25/640)))
		angles = []

		if lines is not None:
			for line in lines:
				for rho, theta in line:
					a = np.cos(theta)
					b = np.sin(theta)
					x0 = a*rho
					y0 = b*rho
					x1 = int(x0 + 1000*(-b))
					y1 = int(y0+1000*(a))
					x2 = int(x0 - 1000*(-b))
					y2 = int(y0 -1000*(a))

					if theta < right_limit_radian and theta > left_limit_radian:
						continue
					else:
						angle = np.arctan((x2-x1)/(y1-y2))*180/np.pi
						angles.append(angle)

						cv2.line(image_original, (x1,y1), (x2,y2), (255,255,255))

		if len(angles) == 0:
			result = 0.
		else:
			result = np.median(angles)

		return result
	except Exception as e:
		_, _, tb = sys.exc_info()
		print(f"gradient detection error = {e}, error line = {tb.tb_lineno}")
		exception_image_path = "./exception_image/"
		try:
			if not os.path.exists(exception_image_path):
				os.mkdir(exception_image_path)
		except OSError:
			print('Error: Creating directory. ' + exception_image_path)
		return 0, None

def warpping(image, srcmat, dstmat):
	(h, w) = (image.shape[0], image.shape[1])
	transform_matrix = cv2.getPerspectiveTransform(srcmat, dstmat)
	minv = cv2.getPerspectiveTransform(dstmat, srcmat)
	_image = cv2.warpPerspective(image, transform_matrix, (w,h))
	return _image, minv

def bird_convert(img, srcmat, dstmat):
	srcmat = np.float32(srcmat)
	dstmat = np.float32(dstmat)
	img_warpped, minverse = warpping(img, srcmat, dstmat)
	return img_warpped

def roi_rectangle_below(img, cutting_idx):
	img = img[cutting_idx:]
	return img

def draw_edge(cv_image: np.array, detection, color: Tuple[int]) -> np.array:
	mask_msg = detection.mask
	mask_array = np.array([[int(ele.x), int(ele.y)] for ele in mask_msg.data])

	if mask_msg.data:
		cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True, color=color, thickness=1, lineType=cv2.LINE_AA)
	return cv_image

def draw_edges(detection_msg, cls_name: str, color: Tuple[int]):
	cv_image = np.zeros((detection_msg.detections[0].mask.height, detection_msg.detections[0].mask.width))
	for detection in detection_msg.detections:
		if detection.class_name == cls_name:
			cv_image = draw_edge(cv_image, detection, color=255)
	return cv_image

def edge_image_postproc(cv_image: np.array, show_image=True):
	(h, w) = (cv_image.shape[0], cv_image.shape[1])
	dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)], [round(w * 0.7), h], [round(w * 0.3), h]]
	src_mat = [[238, 316],[402, 313], [501, 476], [155, 476]]

	lane2_bird_img = bird_convert(cv_image, srcmat=src_mat, dstmat=dst_mat)
	roi_img = roi_rectangle_below(lane2_bird_img, 300)

	if show_image:
		cv_image_names = ['lane2_edge_img', 'lane2_bird_img', 'roi_img']
		cv_image_list = [cv_image, lane2_bird_img, roi_img]
		for name, image in zip(cv_image_names, cv_image_list):
			cv2.imshow(name, image)
		cv2.waitKey(1)

	return roi_img

def get_lane_center(cv_image: np.array, detection_height: int, detection_thickness: int, road_gradient: float, lane_width: int, line_side: str = None, tilt_comp: float = 0.0, force_single_line: bool = False) -> int:
	"""BEV ROI에서 차선 중심 x를 추정한다. 실패하면 -1.

	line_side: 지금 보고 있는 선이 차선의 어느 쪽인지. 'left' | 'right' | None.
	  한쪽 선만 보일 때 중심을 어느 방향으로 밀지 결정한다. None이면 예전처럼
	  road_gradient 부호로 추측하는데, 곡선이나 노이즈에서 부호가 뒤집히면
	  추정 중심이 차선 폭만큼 통째로 반대편으로 튄다. 호출측이 추종 중인
	  클래스(lane_1=좌측선, lane_2=우측선)를 알고 있으므로 넘겨주는 편이 확실하다.

	tilt_comp: 차선 기울기(cos) 보정 강도. 0.0=보정 없음(예전 동작), 1.0=완전 보정.
	  아래 '한쪽 선만 보인 경우' 주석 참고. 보정 배율은 2.0배로 제한된다.

	force_single_line: '두 선이 보인다' 분기를 막을지 여부. 기본값 False=예전 동작.
	  아래 주석 참고. 켜면 lane_width를 반드시 다시 재야 한다.

	tilt_comp / force_single_line 둘 다 기본값이 '예전 동작'이다.
	타겟 위치를 바꾸는 변경이라, lane_width는 이 둘이 꺼진 상태에서 역산된 값이기 때문이다.
	"""
	detection_area_upper_bound = detection_height - int(detection_thickness/2)
	detection_area_lower_bound = detection_height + int(detection_thickness/2)

	image_width = cv_image.shape[1]

	detected_x_coords = np.sort(np.where(cv_image[detection_area_upper_bound:detection_area_lower_bound,:]!=0)[1])

	# 차선 픽셀이 거의 없으면 '측정 불가'다. 예전에는 lane_width/2(=150)를 돌려줬는데,
	# 이건 640폭 영상에서 한참 왼쪽인 좌표라 호출측이 진짜 차선 중심으로 착각했다.
	# (실측: 전체 타겟의 20%가 이 값 150으로 나왔다)
	# -1은 호출측(lane_info_extractor_node, path_planner_node)이 이미 무효로 걸러낸다.
	if (detected_x_coords.shape[0] < 5):
		return -1

	cut_outliers_array = detected_x_coords[1:-1]
	difference_array = cut_outliers_array[1:] - cut_outliers_array[:-1]

	max_diff_idx_left = np.argmax(difference_array)
	max_diff_idx_right = np.argmax(difference_array)+1
	left_val = cut_outliers_array[max_diff_idx_left]
	right_val = cut_outliers_array[max_diff_idx_right]

	# force_single_line: '두 선이 보인다' 분기를 아예 막을지 여부. 기본값 False = 예전 동작.
	#
	# draw_edges()가 추종 클래스 하나만 그리므로 이 함수가 보는 선은 원리상 항상 1개다.
	# 그런데 마스크가 기울거나 조각나면 같은 선의 픽셀이 lane_width/3(=72px) 넘게 벌어져
	# 아래 else로 빠지고, '그 선의 중앙'을 차선 중심으로 반환한다
	# (= 타겟이 폭의 절반인 108px만큼 통째로 이동).
	#
	# 원리만 보면 항상 막는 게 맞지만, 그러면 안 된다.
	# lane_width_for_center(216)는 이 두 분기가 섞인 상태에서 주행 184프레임으로 역산한
	# 값이라, 분기 동작을 바꾸면 그 캘리브레이션이 통째로 무효가 된다.
	# 실제로 이걸 무조건 막았더니 타겟이 108px 밀려 차가 중심으로 복귀하지 못했다.
	# 켜려면 lane_width_for_center를 반드시 다시 재야 한다(README 10번).
	if force_single_line or abs(left_val - right_val) < (lane_width/3):
		line_x_axis_pixel = cut_outliers_array[round((cut_outliers_array.shape[0])/2)]
		center_pixel = None
	else:
		line_x_axis_pixel = None
		center_pixel = (left_val + right_val)/2

	if center_pixel is not None:
		# 양쪽 선이 모두 보인 경우: 두 선의 중점이 곧 차선 중심
		road_target_point_x = center_pixel
	elif line_x_axis_pixel is not None:
		# 한쪽 선만 보인 경우: 차선 폭의 절반만큼 밀어서 중심을 추정.
		# 좌측선을 보고 있으면 중심은 그 오른쪽, 우측선이면 그 왼쪽에 있다.
		#
		# 밀어야 할 방향은 '차선에 수직'인데, 우리는 같은 행(가로)에서 민다.
		# 차선이 수직에서 theta만큼 기울어져 있으면 같은 행에서의 가로 거리는
		# (lane_width/2)/cos(theta)다. cos 보정을 빼면 곡선에서 항상 부족하게 밀려
		# 추정 중심이 '지금 보고 있는 선' 쪽으로 끌려간다.
		#   theta=30도 -> 17px, 40도 -> 33px, 50도 -> 60px 부족
		#   theta=60도 -> 108px 부족 = 폭의 절반, 즉 타겟이 선 위에 정확히 얹힌다
		# lane_2(우측선)를 추종하는 우회전 구간에서는 그 선이 곧 인코스라
		# "곡선에서 인코스 선을 밟고 주행"으로 나타났다.
		#
		# theta는 dominant_gradient()가 이미 구해서 넘겨준 값(도 단위, 수직 기준)이다.
		# 90도 근처에서 발산하므로 배율을 2.0배로 제한한다.
		# 검출 실패 시 dominant_gradient()는 0.0을 주므로 보정 없음(기존 동작)이 된다.
		#
		# 한계: theta는 ROI 전체에 대한 Hough 각도의 중앙값이라 모든 행에 같은 값을 쓴다.
		# 실제로는 곡선에서 가까운 행일수록 덜 기울어져 있어, 근거리는 과보정,
		# 원거리는 부족 보정이 된다. 행별 기울기를 쓰려면 각 행에서 다시 재야 한다.
		tilt_scale = 1.0 / max(np.cos(np.deg2rad(float(road_gradient))), 1e-3)
		tilt_scale = 1.0 + float(tilt_comp) * (tilt_scale - 1.0)
		half_width = (lane_width / 2.0) * min(max(tilt_scale, 1.0), 2.0)

		if line_side == 'left':
			road_target_point_x = line_x_axis_pixel + half_width
		elif line_side == 'right':
			road_target_point_x = line_x_axis_pixel - half_width
		elif road_gradient > 0:
			road_target_point_x = line_x_axis_pixel + half_width
		else:
			road_target_point_x = line_x_axis_pixel - half_width
	else:
		return -1

	global _debug_call_counter
	_debug_call_counter += 1
	if _debug_call_counter % 15 == 0:
		branch = 'TWO_LINE(tilt_comp 미적용!)' if center_pixel is not None else 'single_line(tilt_comp 적용)'
		_rclpy_logging.get_logger('lane_center_debug').info(
			f"[진단] row={detection_height} branch={branch} gap={abs(int(left_val)-int(right_val))}px "
			f"theta={float(road_gradient):.1f} raw_target_x={road_target_point_x:.0f}")

	# 영상 폭으로 clamp한다. 예전에는 lane_width(차선 폭, 기본 300)로 잘랐는데,
	# 차선 폭을 영상 폭인 양 쓴 것이라 640폭 영상에서 타겟이 항상 299 이하로
	# 눌렸다. 즉 중앙(320)보다 무조건 왼쪽이 됐다.
	# (실측: 전체 타겟의 20%가 정확히 299로 고정돼 나왔다)
	return max(0, min(image_width - 1, road_target_point_x))


def get_traffic_light_color(cv_image: np.array, bbox, hsv_ranges: dict) -> str:
	x_min, x_max = int(bbox.center.position.x - bbox.size.x / 2), int(bbox.center.position.x + bbox.size.x / 2)
	y_min, y_max = int(bbox.center.position.y - bbox.size.y / 2), int(bbox.center.position.y + bbox.size.y / 2)
	roi = cv_image[y_min:y_max, x_min:x_max]

	hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

	red_lower1, red_upper1 = hsv_ranges['red1']
	red_lower2, red_upper2 = hsv_ranges['red2']
	yellow_lower, yellow_upper = hsv_ranges['yellow']
	green_lower, green_upper = hsv_ranges['green']

	red_mask1 = cv2.inRange(hsv_roi, red_lower1, red_upper1)
	red_mask2 = cv2.inRange(hsv_roi, red_lower2, red_upper2)
	red_mask = red_mask1 + red_mask2
	yellow_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
	green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)

	red_ratio = cv2.countNonZero(red_mask) / (roi.size / 3)
	yellow_ratio = cv2.countNonZero(yellow_mask) / (roi.size / 3)
	green_ratio = cv2.countNonZero(green_mask) / (roi.size / 3)

	max_ratio = max(red_ratio, yellow_ratio, green_ratio)
	if max_ratio == red_ratio:
		return "Red"
	elif max_ratio == yellow_ratio:
		return "Yellow"
	elif max_ratio == green_ratio:
		return "Green"
	else:
		return "Unknown"



