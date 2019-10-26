#install.packages('plyr')
#install.packages('jsonlite')
#install.packages('ggplot2')

library(plyr)
library(jsonlite)
library(ggplot2)

BYTE_TO_MEGAB_DIVISOR <- 1000000
BYTE_TO_GIGAB_DIVISOR <- 1000000000

readPerformanceFiles <-function(outputDir) {
  #resultDir <- "perfData"
  #outputBaseDir <- "/home/shahin/Desktop/morseLogging/performanceAnalysis"
  #outputDir <- file.path(outputBaseDir, resultDir)
  fileNames <- list.files(path = outputDir)
  perfData <- data.frame()
  data_set_id = 1
  for (jFile in fileNames) {
    fileLocation <- file.path(outputDir, jFile)
    json_data <- fromJSON(txt = fileLocation, flatten = TRUE)
    json_data <- cbind(json_data, data.frame("data_set_id"=jFile))
    perfData <- rbind.fill(perfData, data.frame(json_data))
    data_set_id = + data_set_id + 1
  }
  return(perfData)
}



noLoggingResults <- readPerformanceFiles("~/your_location/dir1")
fullLoggingResults <- readPerformanceFiles("~/your_location/dir2")

noLoggingResults <- cbind(noLoggingResults, data.frame("loggingCategory" = "none"))
fullLoggingResults <- cbind(fullLoggingResults, data.frame("loggingCategory" = "all"))

allResults <- rbind(noLoggingResults,fullLoggingResults)

#
allResults$ros_node_stats.io_writes <- allResults$ros_node_stats.io_writes/BYTE_TO_MEGAB_DIVISOR
allResults$overall_stats.io_writes <- allResults$overall_stats.io_writes/BYTE_TO_MEGAB_DIVISOR

allResults$ros_node_stats.memory_usage <- allResults$ros_node_stats.memory_usage/BYTE_TO_MEGAB_DIVISOR
allResults$overall_stats.memory_usage <- allResults$overall_stats.memory_usage/BYTE_TO_MEGAB_DIVISOR




ggplot(data = allResults, aes(x=loggingCategory, y=ros_node_stats.cpu_usage_percent)) + geom_boxplot() + geom_jitter(width = 0.0) + coord_cartesian(ylim = c(40, 80)) + labs(title="Results for PLACEHOLDER", x ="Logging granularity", y = "CPU usage in %") + theme(text = element_text(size=16), axis.text.x = element_text(angle=0),legend.direction='horizontal', legend.position="bottom", panel.grid.major.x = element_blank(), plot.title = element_text(hjust = 0.5))

ggplot(data = allResults, aes(x=loggingCategory, y=ros_node_stats.memory_usage)) + geom_boxplot() + geom_jitter(width = 0.0) + labs(title="Results for PLACEHOLDER", x ="Logging granularity", y = "CPU usage in %") + theme(text = element_text(size=16), axis.text.x = element_text(angle=0),legend.direction='horizontal', legend.position="bottom", panel.grid.major.x = element_blank(), plot.title = element_text(hjust = 0.5))

