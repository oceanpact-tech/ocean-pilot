#pragma once

#include <string>
#include <stdexcept>

/**
 * @brief Extrai o valor do parâmetro 'seq' de uma string de resposta HTTP.
 * 
 * @param response String completa, ex: "WAYPOINT_ACCEPTED seq=1 Latitude=..."
 * @return int O ID da sequência, ou -1 caso a string seja inválida ou não contenha 'seq='.
 */
inline int extract_waypoint_seq(const std::string& response) 
{
    const std::string key = "seq=";
    size_t pos = response.find(key);
    
    // Verifica se a chave foi encontrada na string
    if (pos != std::string::npos) 
    {
        try {
            // Avança o cursor para logo após o "=" e converte
            // std::stoi para automaticamente no primeiro espaço ou vírgula
            return std::stoi(response.substr(pos + key.length()));
        } 
        catch (const std::invalid_argument&) {
            // Não havia números após 'seq='
            return -1; 
        } 
        catch (const std::out_of_range&) {
            // O número era gigantesco e estourou o limite do int
            return -1; 
        }
    }
    
    return -1; // Padrão "seq=" não encontrado
}