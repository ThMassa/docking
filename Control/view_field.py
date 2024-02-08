import io
import matplotlib.pyplot as plt
import numpy as np
import mplleaflet
from PIL import Image
import requests

def load_background_image(extent):
    """Récupère une image (carte) de la zone définie par extent

    Args:
        extent (list): domaine [lon_min, lat_min, lon_max, lat_max] de la zone considérée

    Returns:
        np.ndarray: image de la carte
    """

    # Download OpenStreetMap image using requests
    url = "https://maps.geoapify.com/v1/staticmap?area=rect:{},{},{},{}&apiKey=43ddc05d66904824999d627ecc27f2c8"

    response = requests.get(url.format(extent[0], extent[1], extent[2], extent[3]))

    # Check if the response is valid
    if response.status_code == 200:
        # Open the image using Pillow
        image = Image.open(io.BytesIO(response.content))
        return np.array(image)
    else:
        print("Failed to download image:", response.status_code)
        return None


def compute_view_rectangle(center, east_width, north_height):
    """Calcule les limites [lon_min, lat_min, lon_max, lat_max] en fonction du centre [lon,lat] et de l'étendue en mètres sur les deux axes

    Args:
        center (list): Centre [lon,lat] de la zone voulue
        east_width (float): étendue en mètres de par et d'autre du centre selon l'axe EST/OUEST
        north_height (float): étendue en mètres de par et d'autre du centre selon l'axe NORD/SUD

    Returns:
        list : domaine [lon_min, lat_min, lon_max, lat_max]
    """
    lon, lat = center
    R = 6480e3
    dlon, dlat = 180/np.pi * east_width/R, 180/np.pi * north_height/R

    rec = [lon - dlon, lat - dlat, lon + dlon, lat + dlat]

    return rec



def plot_map(map, longitude_bounds, latitude_bounds, title, nro_fig=None):
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

## Coordonnées au milieu du lac
lon_center = -3.0152798088714383
lat_center = 48.198741654368305
center = [lon_center, lat_center]

east_width = 100 #demi largeur de l'image en mètres
north_height = 100 #demi hauteur de l'image en mètres
extent = compute_view_rectangle(center, east_width=east_width, north_height=north_height)

## Récupération de la carte de la zone 
background_image = load_background_image(extent)

## Affichage de la carte
plot_map(background_image, [extent[0], extent[2]], [extent[1], extent[3]], "Guerlédan")
plt.show()
