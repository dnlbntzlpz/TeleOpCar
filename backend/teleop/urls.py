from django.urls import path
from .views import index, video_feed_1, video_feed_2, send_command, send_controller_command, get_mock_telemetry

urlpatterns = [
    path('', index, name='index'),
    path('video_feed_1/', video_feed_1, name='video_feed_1'),
    path('video_feed_2/', video_feed_2, name='video_feed_2'),
    path('send_command/', send_command, name='send_command'),
    path('send_controller_command/', send_controller_command, name='send_controller_command'),
    path("get_telemetry/", get_mock_telemetry, name="get_telemetry"),
]
