
import flet as ft

import flet.map as map


def main(page: ft.Page):
    

    marker_layer_ref = ft.Ref[map.MarkerLayer]()
    
    sat_marker = map.Marker(
        content=ft.Icon(ft.icons.LOCATION_ON),
        coordinates=map.MapLatitudeLongitude(30, 15),
    )

    sat_map = map.Map(
        expand=True,
        configuration=map.MapConfiguration(
            initial_center=map.MapLatitudeLongitude(15, 10),
            initial_zoom=4.2,
            interaction_configuration=map.MapInteractionConfiguration(
            flags=map.MapInteractiveFlag.ALL
            ),
        ),
        layers=[
            map.TileLayer(
                url_template="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                on_image_error=lambda e: print("TileLayer Error"),
            ),
            map.MarkerLayer(
                #ref=marker_layer_ref,
                markers=[
                    sat_marker
                ],
            ),
        ],
    )
    page.add(
        ft.Row(
            [
                ft.Container(
                    sat_map,
                    margin=0,
                    padding=0,
                    alignment=ft.alignment.center_left,
                    bgcolor=ft.colors.BLACK,
                    width=500,
                    height=1000,
                    border_radius=0
                )
            ]
            #alignment=ft.MainAxisAlignment.CENTER
        )
        
    )


ft.app(target=main)