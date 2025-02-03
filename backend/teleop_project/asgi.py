"""
ASGI config for teleop_project project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/5.1/howto/deployment/asgi/
"""

import os
from django.core.asgi import get_asgi_application
from django.conf.urls.static import static
from django.conf import settings
from django.urls import path
from teleop.views import video_feed_1, video_feed_2

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'teleop_project.settings')

application = get_asgi_application()
