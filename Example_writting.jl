
# https://en.wikibooks.org/wiki/Introducing_Julia/Working_with_text_files#Writing_to_files
# lien utile

numbers = rand(5,5)

open("romain2.txt", "w") do f
        write(f, "$numbers \n")
end# ne mets aucun espace , il faut boucler et oragniser comme on veut


numbers = rand(5,5)
writedlm("romain1.txt", numbers)#organise comme on veut
