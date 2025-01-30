from django.shortcuts import render

# Create your views here.

def index(request):
    return render(request, 'teleop/index.html') # renders main page