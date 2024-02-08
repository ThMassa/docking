import io
import matplotlib.pyplot as plt
import numpy as np
import mplleaflet
from PIL import Image
import requests

# Function to load OpenStreetMap image as background
def load_background_image(extent):
    # Convert extent to pixel coordinates
    xlim = [extent[0], extent[1]]
    ylim = [extent[2], extent[3]]

    # Download OpenStreetMap image using requests
    url = "https://maps.geoapify.com/v1/staticmap?area=rect:{},{},{},{}&apiKey=43ddc05d66904824999d627ecc27f2c8"

    response = requests.get(url.format(extent[0], extent[1], extent[2], extent[3]))

    # Check if the response is valid
    if response.status_code == 200:
        # Open the image using Pillow
        image = Image.open(io.BytesIO(response.content))
        return np.array(image), (xlim[0], xlim[1], ylim[0], ylim[1])
    else:
        print("Failed to download image:", response.status_code)
        return None, None


def compute_view_rectangle(center, east_width, north_height):
    lon, lat = center
    R = 6480e3
    dlon, dlat = 180/np.pi * east_width/R, 180/np.pi * north_height/R

    rec = [lon - dlon, lat - dlat, lon + dlon, lat + dlat]

    return rec



def plot_map(map, longitude_bounds, latitude_bounds,
             title, nro_fig=None):
    if nro_fig is None:
        ff = plt.figure()
    else:
        ff = plt.figure(nro_fig)
    ff.clf()
    gg = ff.add_subplot(111)

    gg.imshow(map, extent=[longitude_bounds[0], longitude_bounds[1],
                           latitude_bounds[0], latitude_bounds[1]],
              aspect="equal")

    gg.set_xlabel("longitude (°)")
    gg.set_ylabel("latitude (°)")
    gg.set_title(title)
    gg.grid(True)
    #plt.pause(0.01)
    return gg

lon_center = -3.0152798088714383
lat_center = 48.198741654368305
center = [lon_center, lat_center]
offset = 0.01

east_width = 100
north_height = 100

extent = [lon_center - offset, lat_center - offset, lon_center + offset, lat_center + offset]
extent = compute_view_rectangle(center, east_width=east_width, north_height=north_height)

background_image, extent = load_background_image(extent)
plot_map(background_image, [extent[0], extent[2]], [extent[1], extent[3]], "Guerlédan")
plt.show()
