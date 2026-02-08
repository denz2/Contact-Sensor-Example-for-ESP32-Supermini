idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_matter # Ensure this is an underscore
)

# Use the component name here
idf_component_get_property(matter_lib esp_matter lib) 
target_link_libraries(${COMPONENT_LIB} PUBLIC ${matter_lib})


