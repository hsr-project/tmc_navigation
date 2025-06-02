comp_select_map () {
    local map_path cur
    map_path=$(rospack find tmc_potential_maps)
    if [ ! -d ${map_path}/maps ] ; then
	return
    fi
    cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=($(compgen -W "$(ls ${map_path}/maps/)" ${cur}))
}

alias select_map='rosrun tmc_grid_map_server select_map'
complete -F comp_select_map select_map
