import io
import matplotlib.pyplot as plt
import numpy as np
import mplleaflet
from PIL import Image
import requests

# Function to load OpenStreetMap image as background
def load_background_image(extent, zoom=13):
    # Convert extent to pixel coordinates
    xlim = [extent[0], extent[1]]
    ylim = [extent[2], extent[3]]
    KEY = "W80eEWQhF5WmSF19oWcmXh113xWTeH4JGfDyf51W"
    width, height = 640, int(640 * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0]))
    lonlat = "-3.0152798088714383,48.198741654368305"
    zoom = 17

    # Download OpenStreetMap image using requests
    url = "https://api.nasa.gov/planetary/earth/imagery?lon=-3.75&lat=48&date=2014-01-01&dim=1&api_key={}"
    url = "https://maps.geoapify.com/v1/staticmap?center=lonlat:{}&zoom={}&apiKey=43ddc05d66904824999d627ecc27f2c8"
    xtile = int((xlim[0] + 180) / 360 * 2 ** zoom)
    ytile = int((1 - np.log(np.tan(np.radians(ylim[0])) + 1 / np.cos(np.radians(ylim[0]))) / np.pi) / 2 * 2 ** zoom)
    response = requests.get(url.format(lonlat, zoom))

    # Check if the response is valid
    if response.status_code == 200:
        # Open the image using Pillow
        image = Image.open(io.BytesIO(response.content))
        return np.array(image), (xlim[0], xlim[1], ylim[0], ylim[1])
    else:
        print("Failed to download image:", response.status_code)
        return None, None


# Plot the data
plt.figure(figsize=(10, 8))
plt.title('Guerlédan')
background_image, extent = load_background_image([min(longs), max(longs), min(lats), max(lats)])
# print(background_image.shape)
plt.imshow(background_image, extent=extent, origin='upper')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.show()

# Convert to interactive leaflet map
# mplleaflet.show()
