file(GLOB FILES *.jpg *.png *.yml *.json)

foreach(file ${FILES})
  get_filename_component(filename ${file} NAME) 
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${filename} ${CMAKE_CURRENT_BINARY_DIR}/${filename} COPYONLY)
endforeach(file)
